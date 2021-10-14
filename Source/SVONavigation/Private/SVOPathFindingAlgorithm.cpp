
#include "SVOPathFindingAlgorithm.h"

#include "SVONavigationData.h"
#include "SVONavigationQueryFilterImpl.h"
#include "SVONavigationTypes.h"
#include "SVOPathCostCalculator.h"
#include "SVOPathHeuristicCalculator.h"
#include "Kismet/GameplayStatics.h"

namespace
{
    void ApplyVerticalOffset( FNavigationPath & path, const float vertical_offset )
    {
        auto & path_points = path.GetPathPoints();

        for (auto & path_point : path_points)
        {
            path_point.Location.Z += vertical_offset;
        }
    }

    void BuildPath( FNavigationPath & path, const FSVOPathFindingParameters & params, const TMap< FSVOLinkWithLocation, FSVOLinkWithLocation > & came_from )
    {
        auto & path_points = path.GetPathPoints();

        FSVOLinkWithLocation current( params.EndLink, *params.BoundsNavigationData );

        const auto get_link_position = []( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & link ) {
            FVector link_position;
            bounds_data.GetLinkPosition( link_position, link );
            return link_position;
        };

        const auto & bounds_data = *params.BoundsNavigationData;

        path_points.Add( params.EndLocation );

        while (current.Link != params.StartLink)
        {
            current = came_from[ current ];
            path_points.Emplace( current.Location );
        }

        //path_points.Emplace( get_link_position( bounds_data, start_link ) );
        path_points.Emplace( params.StartLocation );

        Algo::Reverse( path_points );
    }

    void AdjustPathEnds( FNavigationPath & path, const FVector & start_location, const FVector & end_location )
    {
        auto & path_points = path.GetPathPoints();
        path_points[ 0 ].Location = start_location;

        if (path_points.Num() > 1)
        {
            path_points.Top().Location = end_location;
        }
    }
}

FSVOLinkWithLocation::FSVOLinkWithLocation( const FSVOOctreeLink & link, const FVector & location ) :
        Link( link ),
        Location( location )
{
}

FSVOLinkWithLocation::FSVOLinkWithLocation( const FSVOOctreeLink & link, const FSVOBoundsNavigationData & bounds_navigation_data ) :
    Link( link )
{
    bounds_navigation_data.GetLinkPosition( Location, link );
}

void FSVOPathFinderDebugCost::Reset()
{
    *this = FSVOPathFinderDebugCost();
}

void FSVOPathFinderDebugStep::Reset()
{
    CurrentLocationCost.Reset();
    NeighborLocationCosts.Reset();
}

void FSVOPathFinderDebugInfos::Reset()
{
    DebugSteps.Reset();
    CurrentBestPath.ResetForRepath();
    Iterations = 0;
    VisitedNodes = 0;
}

FSVOPathFindingParameters::FSVOPathFindingParameters( const FNavAgentProperties & agent_properties, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query ) :
    AgentProperties( agent_properties ),
    NavigationData( navigation_data ),
    StartLocation( start_location ),
    EndLocation( end_location ),
    NavigationQueryFilter( *path_finding_query.QueryFilter ),
    QueryFilterImplementation( static_cast< const FSVONavigationQueryFilterImpl * >( path_finding_query.QueryFilter->GetImplementation() ) ),
    QueryFilterSettings( QueryFilterImplementation->QueryFilterSettings ),
    HeuristicCalculator( QueryFilterSettings.PathHeuristicCalculatorClass->GetDefaultObject<USVOPathHeuristicCalculator>() ),
    CostCalculator( QueryFilterSettings.PathCostCalculatorClass->GetDefaultObject<USVOPathCostCalculator>() ),
    BoundsNavigationData( navigation_data.GetSVOData().GetBoundsNavigationDataContainingPoints( { start_location, end_location } ) ),
    VerticalOffset( QueryFilterSettings.bOffsetPathVerticallyByAgentRadius ? -path_finding_query.NavAgentProperties.AgentRadius : 0.0f )
{
    if ( BoundsNavigationData != nullptr )
    {
        BoundsNavigationData->GetLinkFromPosition( StartLink, StartLocation );
        BoundsNavigationData->GetLinkFromPosition( EndLink, EndLocation );
    }
}

template <typename _ALGO_>
TSVOPathFindingAlgorithmObserver<_ALGO_>::TSVOPathFindingAlgorithmObserver( _ALGO_ & algo ) :
    Algo( algo )
{
}

FSVOPathFindingAlgorithmStepper::FSVOPathFindingAlgorithmStepper( const FSVOPathFindingParameters & params ) :
    Params( params )
{
}

void FSVOPathFindingAlgorithmStepper::AddObserver( TSharedPtr<FSVOPathFindingAlgorithmObserver> observer )
{
    Observers.Add( observer );
}

FSVOPathFindingAlgorithmStepper_AStar::FSVOPathFindingAlgorithmStepper_AStar( const FSVOPathFindingParameters & params ):
    FSVOPathFindingAlgorithmStepper( params )
{
    if ( params.StartLink.IsValid() && params.EndLink.IsValid() )
    {
        FSVOLinkWithLocation link_with_location( params.StartLink, params.StartLocation );
        OpenSet.Emplace( link_with_location, params.HeuristicCalculator->GetHeuristicCost( *params.BoundsNavigationData, params.StartLink, params.EndLink ) * params.NavigationQueryFilter.GetHeuristicScale() );
        CameFrom.Add( link_with_location, link_with_location );
        CostSoFar.Add( params.StartLink, 0.0f );        
    }
}

bool FSVOPathFindingAlgorithmStepper_AStar::Step( ENavigationQueryResult::Type & result )
{
    if ( Params.BoundsNavigationData == nullptr
        || !Params.StartLink.IsValid()
        || !Params.EndLink.IsValid()
        || OpenSet.Num() == 0
        )
    {
        result = ENavigationQueryResult::Fail;
        return false;
    }

    const auto current = OpenSet.Pop();

    for ( const auto & observer : Observers )
    {
        observer->OnOpenNode( current );   
    }

    if ( current.LinkWithLocation.Link == Params.EndLink )
    {
        result = ENavigationQueryResult::Success;
        for ( const auto & observer : Observers )
        {
            observer->OnEndLinkReached();   
        }
        return false;
    }

    TArray< FSVOOctreeLink > neighbors;
    Params.BoundsNavigationData->GetNeighbors( neighbors, current.LinkWithLocation.Link );

    for ( const auto & neighbor : neighbors )
    {
        ProcessNeighbor( current.LinkWithLocation, neighbor );
    }

    for ( const auto & observer : Observers )
    {
        observer->OnEndStep();   
    }

    return true;
}

void FSVOPathFindingAlgorithmStepper_AStar::ProcessNeighbor( FSVOLinkWithLocation current, FSVOOctreeLink neighbor )
{
    const FSVOLinkWithLocation neighbor_link_with_location( neighbor, *Params.BoundsNavigationData );
    const auto neighbor_cost = Params.CostCalculator->GetCost( *Params.BoundsNavigationData, current.Link, neighbor );
    const auto new_cost = CostSoFar[ current.Link ] + neighbor_cost;

    for ( const auto & observer : Observers )
    {
        observer->OnNeighborCostComputed( neighbor, neighbor_cost );   
    }

    if (!CostSoFar.Contains( neighbor ) || new_cost < CostSoFar[ neighbor ])
    {
        CostSoFar.FindOrAdd( neighbor ) = new_cost;
        const auto priority = new_cost + Params.HeuristicCalculator->GetHeuristicCost( *Params.BoundsNavigationData, neighbor, Params.EndLink ) * Params.NavigationQueryFilter.GetHeuristicScale();
        OpenSet.Emplace( neighbor_link_with_location, priority );
        OpenSet.Sort();
        CameFrom.FindOrAdd( neighbor_link_with_location ) = current;
            
        for ( const auto & observer : Observers )
        {
            observer->OnFoundBetterNeighbor();   
        }
    }

    for ( const auto & observer : Observers )
    {
        observer->OnNeighborVisited();   
    }
}

FSVOPathFindingAlgorithmStepper_ThetaStar::FSVOPathFindingAlgorithmStepper_ThetaStar( const FSVOPathFindingParameters & params, const FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters & theta_params ) :
    FSVOPathFindingAlgorithmStepper_AStar( params ),
    ThetaStarParams( theta_params )
{
}

void FSVOPathFindingAlgorithmStepper_ThetaStar::ProcessNeighbor( FSVOLinkWithLocation current, FSVOOctreeLink neighbor )
{
    const auto parent_current_link = CameFrom[ current ];
        
    if ( IsInLineOfSight( parent_current_link, neighbor ) )
    {
        FSVOPathFindingAlgorithmStepper_AStar::ProcessNeighbor( parent_current_link, neighbor );
    }
    else
    {
        FSVOPathFindingAlgorithmStepper_AStar::ProcessNeighbor( current, neighbor );
    }
}

bool FSVOPathFindingAlgorithmStepper_ThetaStar::IsInLineOfSight( const FSVOLinkWithLocation & from, FSVOOctreeLink to ) const
{
    auto * world = Params.NavigationData.GetWorld();

    if ( !ensure( world != nullptr ) )
    {
        return false;
    }
    
    FVector to_position;
    if ( !Params.BoundsNavigationData->GetLinkPosition( to_position, to ) )
    {
        return false;
    }

    FHitResult hit_result;
    return !UKismetSystemLibrary::SphereTraceSingle(
        world,
        from.Location,
        to_position,
        /* Params.AgentProperties.AgentRadius */ 50.0f * ThetaStarParams.AgentRadiusMultiplier,
        //UEngineTypes::ConvertToTraceType( Params.BoundsNavigationData->GetDataGenerationSettings().GenerationSettings.CollisionChannel ),
        ThetaStarParams.TraceType,
        false,
        TArray< AActor* >(),
        ThetaStarParams.bShowLineOfSightTraces ? EDrawDebugTrace::ForDuration : EDrawDebugTrace::None,
        hit_result,
        false
        );
}

FSVOPathFindingAStarObserver_BuildPath::FSVOPathFindingAStarObserver_BuildPath( FNavigationPath & navigation_path, FSVOPathFindingAlgorithmStepper_AStar & algo ) :
    TSVOPathFindingAlgorithmObserver< FSVOPathFindingAlgorithmStepper_AStar >( algo ),
    NavigationPath( navigation_path )
{
}

void FSVOPathFindingAStarObserver_BuildPath::OnEndLinkReached()
{
    const auto & params = Algo.GetParams(); 
    
    BuildPath( NavigationPath, params, Algo.GetCameFrom() );
    AdjustPathEnds( NavigationPath, params.StartLocation, params.EndLocation );

    if ( params.VerticalOffset != 0.0f )
    {
        ApplyVerticalOffset( NavigationPath, params.VerticalOffset );
    }
    
    NavigationPath.MarkReady();
}

FSVOPathFindingAStarObserver_GenerateDebugInfos::FSVOPathFindingAStarObserver_GenerateDebugInfos( FSVOPathFinderDebugInfos & debug_infos, FSVOPathFindingAlgorithmStepper_AStar & algo ):
    TSVOPathFindingAlgorithmObserver< FSVOPathFindingAlgorithmStepper_AStar >( algo ),
    DebugInfos( debug_infos )
{
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnOpenNode( const FSVOLinkWithCost & link_with_cost )
{
    CurrentLink = link_with_cost.LinkWithLocation.Link;
    
    DebugStep.Reset();
    DebugStep.CurrentLocationCost.Cost = link_with_cost.Cost;
    DebugStep.CurrentLocationCost.Location = link_with_cost.LinkWithLocation.Location;
    //Algo.GetParams().BoundsNavigationData->GetLinkPosition( DebugStep.CurrentLocationCost.Location, link_with_cost.LinkWithLocation.Link );    
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnNeighborCostComputed( const FSVOOctreeLink & link, float cost )
{
    CurrentNeighborDebugCost.Cost = cost;
    Algo.GetParams().BoundsNavigationData->GetLinkPosition( CurrentNeighborDebugCost.Location, link );
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnEndLinkReached()
{
    const auto & params = Algo.GetParams();

    DebugInfos.CurrentBestPath.ResetForRepath();
    BuildPath( DebugInfos.CurrentBestPath, params, Algo.GetCameFrom() );
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnNeighborVisited()
{
    DebugStep.NeighborLocationCosts.Emplace( MoveTemp( CurrentNeighborDebugCost ) );
    DebugInfos.VisitedNodes++;
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnFoundBetterNeighbor()
{
    const auto & params = Algo.GetParams();
    
    // DebugInfos.CurrentBestPath.ResetForRepath();
    // BuildPath( DebugInfos.CurrentBestPath, params, CurrentLink, Algo.GetCameFrom() );
    
    CurrentNeighborDebugCost.WasEvaluated = true;
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnEndStep()
{
    DebugInfos.DebugSteps.Emplace( MoveTemp( DebugStep ) );
    DebugInfos.Iterations++;
}

ENavigationQueryResult::Type USVOPathFindingAlgorithm::GetPath( FNavigationPath & /*navigation_path*/, const FSVOPathFindingParameters & /*params*/ ) const
{
    return ENavigationQueryResult::Error;
}

TSharedPtr<FSVOPathFindingAlgorithmStepper> USVOPathFindingAlgorithm::GetDebugPathStepper( FSVOPathFinderDebugInfos & /*debug_infos*/, const FSVOPathFindingParameters /*params*/ ) const
{
    return nullptr;
}

ENavigationQueryResult::Type USVOPathFindingAlgorithmAStar::GetPath( FNavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const
{
    FSVOPathFindingAlgorithmStepper_AStar stepper_a_star( params );
    const auto path_builder = MakeShared< FSVOPathFindingAStarObserver_BuildPath >( navigation_path, stepper_a_star );

    stepper_a_star.AddObserver( path_builder );
        
    int iterations = 0;

    ENavigationQueryResult::Type result = ENavigationQueryResult::Fail;

    while ( stepper_a_star.Step( result ) )
    {
        iterations++;
    }

    return result;
}
    
TSharedPtr< FSVOPathFindingAlgorithmStepper > USVOPathFindingAlgorithmAStar::GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const
{
    auto stepper_a_star = MakeShared< FSVOPathFindingAlgorithmStepper_AStar > ( params );
    const auto debug_path = MakeShared< FSVOPathFindingAStarObserver_GenerateDebugInfos >( debug_infos, stepper_a_star.Get() );
    stepper_a_star->AddObserver( debug_path );

    return stepper_a_star;
}

ENavigationQueryResult::Type USVOPathFindingAlgorithmThetaStar::GetPath( FNavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const
{
    FSVOPathFindingAlgorithmStepper_ThetaStar stepper_a_star( params, Parameters );
    const auto path_builder = MakeShared< FSVOPathFindingAStarObserver_BuildPath >( navigation_path, stepper_a_star );

    stepper_a_star.AddObserver( path_builder );
        
    int iterations = 0;

    ENavigationQueryResult::Type result = ENavigationQueryResult::Fail;

    while ( stepper_a_star.Step( result ) )
    {
        iterations++;
    }

    return result;
}

TSharedPtr<FSVOPathFindingAlgorithmStepper> USVOPathFindingAlgorithmThetaStar::GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const
{
    auto stepper_a_star = MakeShared< FSVOPathFindingAlgorithmStepper_ThetaStar > ( params, Parameters );
    const auto debug_path = MakeShared< FSVOPathFindingAStarObserver_GenerateDebugInfos >( debug_infos, stepper_a_star.Get() );
    stepper_a_star->AddObserver( debug_path );

    return stepper_a_star;
}

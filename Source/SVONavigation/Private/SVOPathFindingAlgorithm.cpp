
#include "SVOPathFindingAlgorithm.h"

#include "SVONavigationData.h"
#include "SVONavigationQueryFilterImpl.h"
#include "SVONavigationTypes.h"
#include "SVOPathCostCalculator.h"
#include "SVOPathHeuristicCalculator.h"

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

    void BuildPath( FNavigationPath & path, const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start_link, const FSVOOctreeLink & end_link, const TMap< FSVOOctreeLink, FSVOOctreeLink > & came_from )
    {
        auto & path_points = path.GetPathPoints();

        auto current = end_link;

        const auto get_link_position = []( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & link ) {
            FVector link_position;
            bounds_data.GetLinkPosition( link_position, link );
            return link_position;
        };

        while (current != start_link)
        {
            current = came_from[ current ];
            path_points.Emplace( get_link_position( bounds_data, current ) );
        }

        path_points.Emplace( get_link_position( bounds_data, start_link ) );

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

FSVOPathFindingParameters::FSVOPathFindingParameters( const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query ) :
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
        Frontier.Emplace( params.StartLink, params.HeuristicCalculator->GetHeuristicCost( *params.BoundsNavigationData, params.StartLink, params.EndLink ) * params.NavigationQueryFilter.GetHeuristicScale() );
        CameFrom.Add( params.StartLink, params.StartLink );
        CostSoFar.Add( params.StartLink, 0.0f );        
    }
}

bool FSVOPathFindingAlgorithmStepper_AStar::Step( ENavigationQueryResult::Type & result )
{
    if ( Params.BoundsNavigationData == nullptr
        || !Params.StartLink.IsValid()
        || !Params.EndLink.IsValid()
        || Frontier.Num() == 0
        )
    {
        result = ENavigationQueryResult::Fail;
        return false;
    }

    const auto current = Frontier.Pop();

    for ( const auto & observer : Observers )
    {
        observer->OnOpenNode( current );   
    }

    if (current.Link == Params.EndLink)
    {
        result = ENavigationQueryResult::Success;
        for ( const auto & observer : Observers )
        {
            observer->OnEndLinkReached();   
        }
        return false;
    }

    TArray< FSVOOctreeLink > neighbors;
    Params.BoundsNavigationData->GetNeighbors( neighbors, current.Link );

    for (const auto & neighbor : neighbors)
    {
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
            Frontier.Emplace( neighbor, priority );
            Frontier.Sort();
            CameFrom.FindOrAdd( neighbor ) = current.Link;
            
            for ( const auto & observer : Observers )
            {
                observer->OnFoundBetterNeighbor();   
            }
        }

        for ( const auto & observer : Observers )
        {
            observer->OnNeighborVisited();   
        }

        //Visitor.OnNeighborExpanded();
        // debug_step.NeighborLocationCosts.Emplace( MoveTemp( CurrentNeighborDebugCost ) );
        // DebugInfos.VisitedNodes++;
    }

    for ( const auto & observer : Observers )
    {
        observer->OnEndStep();   
    }

    // DebugInfos.DebugSteps.Emplace( MoveTemp( debug_step ) );
    // DebugInfos.Iterations++;

    return true;
}

FSVOPathFindingAStarObserver_BuildPath::FSVOPathFindingAStarObserver_BuildPath( FNavigationPath & navigation_path, FSVOPathFindingAlgorithmStepper_AStar & algo ) :
    TSVOPathFindingAlgorithmObserver< FSVOPathFindingAlgorithmStepper_AStar >( algo ),
    NavigationPath( navigation_path )
{
}

void FSVOPathFindingAStarObserver_BuildPath::OnEndLinkReached()
{
    const auto & params = Algo.GetParams(); 
    
    BuildPath( NavigationPath, *params.BoundsNavigationData, params.StartLink, params.EndLink, Algo.GetCameFrom() );
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
    CurrentLink = link_with_cost.Link;
    
    DebugStep.Reset();
    DebugStep.CurrentLocationCost.Cost = link_with_cost.Cost;
    Algo.GetParams().BoundsNavigationData->GetLinkPosition( DebugStep.CurrentLocationCost.Location, link_with_cost.Link );    
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnNeighborCostComputed( const FSVOOctreeLink & link, float cost )
{
    CurrentNeighborDebugCost.Cost = cost;
    Algo.GetParams().BoundsNavigationData->GetLinkPosition( CurrentNeighborDebugCost.Location, link );
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnEndLinkReached()
{
    
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnNeighborVisited()
{
    DebugStep.NeighborLocationCosts.Emplace( MoveTemp( CurrentNeighborDebugCost ) );
    DebugInfos.VisitedNodes++;
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnFoundBetterNeighbor()
{
    const auto & params = Algo.GetParams();
    
    DebugInfos.CurrentBestPath.ResetForRepath();
    BuildPath( DebugInfos.CurrentBestPath, *params.BoundsNavigationData, params.StartLink, CurrentLink, Algo.GetCameFrom() );
    
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
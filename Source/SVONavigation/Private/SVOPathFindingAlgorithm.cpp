#include "SVOPathFindingAlgorithm.h"

#include "SVOHeuristicCalculator.h"
#include "SVONavigationData.h"
#include "SVONavigationQueryFilterImpl.h"
#include "SVONavigationTypes.h"
#include "SVOTraversalCostCalculator.h"

#include <Kismet/GameplayStatics.h>

namespace
{
    void ApplyVerticalOffset( FNavigationPath & path, const float vertical_offset )
    {
        auto & path_points = path.GetPathPoints();

        for ( auto & path_point : path_points )
        {
            path_point.Location.Z += vertical_offset;
        }
    }

    void BuildPath( FNavigationPath & path, const FSVOPathFindingParameters & params, const TArray< FSVOOctreeLink > & link_path, const bool add_end_location )
    {
        auto & path_points = path.GetPathPoints();
        const auto path_points_size = link_path.Num() + 1;

        path_points.Reset( path_points_size );

        path_points.Emplace( params.StartLocation );

        const auto & bounds_data = *params.BoundsNavigationData;

        for ( auto index = 0; index < link_path.Num() - 1; index++ )
        {
            const auto link = link_path[ index ];
            path_points.Emplace( bounds_data.GetLinkPosition( link ) );
        }

        if ( add_end_location )
        {
            path_points.Emplace( params.EndLocation );
        }
    }

    ENavigationQueryResult::Type GraphAStarResultToNavigationTypeResult( const EGraphAStarResult result )
    {
        constexpr ENavigationQueryResult::Type result_conversion_table[] = {
            ENavigationQueryResult::Fail,
            ENavigationQueryResult::Success,
            ENavigationQueryResult::Fail,
            ENavigationQueryResult::Fail
        };

        return result_conversion_table[ static_cast< int >( result ) ];
    }
}

FSVOLinkWithLocation::FSVOLinkWithLocation( const FSVOOctreeLink & link, const FVector & location ) :
    Link( link ),
    Location( location )
{}

FSVOLinkWithLocation::FSVOLinkWithLocation( const FSVOOctreeLink & link, const FSVOBoundsNavigationData & bounds_navigation_data ) :
    Link( link ),
    Location( bounds_navigation_data.GetLinkPosition( link ) )
{}

FSVOPathFinderDebugNodeCost::FSVOPathFinderDebugNodeCost( const FSVOLinkWithLocation & from, const FSVOLinkWithLocation & to, const float cost, const bool is_closed ) :
    From( from ),
    To( to ),
    Cost( cost ),
    bIsClosed( is_closed )
{
}

void FSVOPathFinderDebugNodeCost::Reset()
{
    *this = FSVOPathFinderDebugNodeCost();
}

void FSVOPathFinderDebugInfos::Reset()
{
    LastProcessedSingleNode.Reset();
    ProcessedNeighbors.Reset();
    Iterations = 0;
    VisitedNodes = 0;
    CurrentBestPath.ResetForRepath();
}

FSVOPathFindingParameters::FSVOPathFindingParameters( const FNavAgentProperties & agent_properties, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query ) :
    AgentProperties( agent_properties ),
    NavigationData( navigation_data ),
    StartLocation( start_location ),
    EndLocation( end_location ),
    NavigationQueryFilter( *path_finding_query.QueryFilter ),
    QueryFilterImplementation( static_cast< const FSVONavigationQueryFilterImpl * >( path_finding_query.QueryFilter->GetImplementation() ) ),
    QueryFilterSettings( QueryFilterImplementation->QueryFilterSettings ),
    HeuristicCalculator( QueryFilterSettings.HeuristicCalculator ),
    CostCalculator( QueryFilterSettings.TraversalCostCalculator ),
    BoundsNavigationData( navigation_data.GetSVOData().GetBoundsNavigationDataContainingPoints( { start_location, end_location } ) ),
    VerticalOffset( QueryFilterSettings.bOffsetPathVerticallyByAgentRadius ? -path_finding_query.NavAgentProperties.AgentRadius : 0.0f )
{
    if ( BoundsNavigationData != nullptr )
    {
        BoundsNavigationData->GetLinkFromPosition( StartLink, StartLocation );
        BoundsNavigationData->GetLinkFromPosition( EndLink, EndLocation );
    }
}

FSVOPathFindingAlgorithmObserver::FSVOPathFindingAlgorithmObserver( const FSVOPathFindingAlgorithmStepper & stepper ) :
    Stepper( stepper )
{
}

FSVOPathFindingAStarObserver_BuildPath::FSVOPathFindingAStarObserver_BuildPath( FNavigationPath & navigation_path, const FSVOPathFindingAlgorithmStepper & stepper ) :
    FSVOPathFindingAlgorithmObserver( stepper ),
    NavigationPath( navigation_path )
{
}

void FSVOPathFindingAStarObserver_BuildPath::OnSearchSuccess( const TArray< FSVOOctreeLink > & link_path )
{
    const auto & params = Stepper.GetParameters();

    BuildPath( NavigationPath, params, link_path, true );

    if ( params.VerticalOffset != 0.0f )
    {
        ApplyVerticalOffset( NavigationPath, params.VerticalOffset );
    }

    NavigationPath.MarkReady();
}

FSVOPathFindingAStarObserver_GenerateDebugInfos::FSVOPathFindingAStarObserver_GenerateDebugInfos( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingAlgorithmStepper & stepper ) :
    FSVOPathFindingAlgorithmObserver( stepper ),
    DebugInfos( debug_infos )
{
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnProcessSingleNode( const FGraphAStarDefaultNode< FSVOBoundsNavigationData > & node )
{
    if ( node.ParentRef.IsValid() )
    {
        DebugInfos.LastProcessedSingleNode.From = FSVOLinkWithLocation( node.ParentRef, *Stepper.GetParameters().BoundsNavigationData );
    }
    else
    {
        DebugInfos.LastProcessedSingleNode.From = FSVOLinkWithLocation( FSVOOctreeLink::InvalidLink(), Stepper.GetParameters().StartLocation );
    }

    DebugInfos.LastProcessedSingleNode.To = FSVOLinkWithLocation( node.NodeRef, *Stepper.GetParameters().BoundsNavigationData );
    DebugInfos.LastProcessedSingleNode.Cost = node.TotalCost;

    DebugInfos.ProcessedNeighbors.Reset();

    DebugInfos.Iterations++;

    TArray< FSVOOctreeLink > link_path;
    Stepper.FillLinkPath( link_path );

    // Fill DebugInfos.CurrentBestPath
    FillCurrentBestPath( link_path, false );
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnProcessNeighbor( const FGraphAStarDefaultNode< FSVOBoundsNavigationData > & parent, const FGraphAStarDefaultNode< FSVOBoundsNavigationData > & neighbor, const float cost )
{
    DebugInfos.ProcessedNeighbors.Emplace( FSVOLinkWithLocation( parent.NodeRef, *Stepper.GetParameters().BoundsNavigationData ), FSVOLinkWithLocation( neighbor.NodeRef, *Stepper.GetParameters().BoundsNavigationData ), cost, true );
    DebugInfos.VisitedNodes++;
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnProcessNeighbor( const FGraphAStarDefaultNode< FSVOBoundsNavigationData > & neighbor )
{
    DebugInfos.ProcessedNeighbors.Emplace( FSVOLinkWithLocation( neighbor.ParentRef, *Stepper.GetParameters().BoundsNavigationData ), FSVOLinkWithLocation( neighbor.NodeRef, *Stepper.GetParameters().BoundsNavigationData ), neighbor.TotalCost, false );
    DebugInfos.VisitedNodes++;
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnSearchSuccess( const TArray< FSVOOctreeLink > & link_path )
{
    FillCurrentBestPath( link_path, true );

    auto & nav_path_points = DebugInfos.CurrentBestPath.GetPathPoints();
    DebugInfos.PathSegmentCount = nav_path_points.Num();

    auto path_length = 0.0f;

    for ( auto index = 1; index < nav_path_points.Num(); index++ )
    {
        const auto segment_length = FVector::Dist( nav_path_points[ index ].Location, nav_path_points[ index - 1 ].Location );
        path_length += segment_length;
    }

    DebugInfos.PathLength = path_length;
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::FillCurrentBestPath( const TArray< FSVOOctreeLink > & link_path, const bool add_end_location ) const
{
    DebugInfos.CurrentBestPath.ResetForRepath();

    const auto & params = Stepper.GetParameters();

    BuildPath( DebugInfos.CurrentBestPath, params, link_path, add_end_location );

    if ( params.VerticalOffset != 0.0f )
    {
        ApplyVerticalOffset( DebugInfos.CurrentBestPath, params.VerticalOffset );
    }

    DebugInfos.CurrentBestPath.MarkReady();
}

FSVOGraphAStar::FSVOGraphAStar( const FSVOBoundsNavigationData & graph ) :
    FGraphAStar< FSVOBoundsNavigationData, FGraphAStarDefaultPolicy, FGraphAStarDefaultNode< FSVOBoundsNavigationData > >( graph )
{
}

FSVOPathFindingAlgorithmStepper::FSVOPathFindingAlgorithmStepper( const FSVOPathFindingParameters & parameters ) :
    Graph( *parameters.BoundsNavigationData ),
    State( ESVOPathFindingAlgorithmState::Init ),
    Parameters( parameters )
{
}

void FSVOPathFindingAlgorithmStepper::AddObserver( const TSharedPtr< FSVOPathFindingAlgorithmObserver > observer )
{
    Observers.Add( observer );
}

ESVOPathFindingAlgorithmStepperStatus FSVOPathFindingAlgorithmStepper::Step( EGraphAStarResult & result )
{
    switch ( State )
    {
        case ESVOPathFindingAlgorithmState::Init:
        {
            return Init( result );
        }
        case ESVOPathFindingAlgorithmState::ProcessNode:
        {
            return ProcessSingleNode( result );
        }
        case ESVOPathFindingAlgorithmState::ProcessNeighbor:
        {
            return ProcessNeighbor();
        }
        case ESVOPathFindingAlgorithmState::Ended:
        {
            return Ended( result );
        }
        default:
        {
            checkNoEntry();
            return ESVOPathFindingAlgorithmStepperStatus::IsStopped;
        }
    }
}

bool FSVOPathFindingAlgorithmStepper::FillLinkPath( TArray< FSVOOctreeLink > & link_path ) const
{
    checkNoEntry();
    return false;
}

void FSVOPathFindingAlgorithmStepper::SetState( const ESVOPathFindingAlgorithmState new_state )
{
    State = new_state;
}

float FSVOPathFindingAlgorithmStepper::GetHeuristicCost( const FSVOOctreeLink & from, const FSVOOctreeLink & to ) const
{
    return Parameters.HeuristicCalculator->GetHeuristicCost( *Parameters.BoundsNavigationData, from, to ) * Parameters.NavigationQueryFilter.GetHeuristicScale();
}

float FSVOPathFindingAlgorithmStepper::GetTraversalCost( const FSVOOctreeLink & from, const FSVOOctreeLink & to ) const
{
    return Parameters.CostCalculator->GetTraversalCost( *Parameters.BoundsNavigationData, from, to );
}

FSVOPathFindingAlgorithmStepper_AStar::FSVOPathFindingAlgorithmStepper_AStar( const FSVOPathFindingParameters & parameters ) :
    FSVOPathFindingAlgorithmStepper( parameters ),
    ConsideredNodeIndex( INDEX_NONE ),
    BestNodeIndex( INDEX_NONE ),
    BestNodeCost( -1.0f ),
    NeighborIndex( INDEX_NONE )
{
}

bool FSVOPathFindingAlgorithmStepper_AStar::FillLinkPath( TArray< FSVOOctreeLink > & link_path ) const
{
    int32 search_node_index = BestNodeIndex;
    int32 path_length = 0;
    do
    {
        path_length++;
        search_node_index = Graph.NodePool[ search_node_index ].ParentNodeIndex;
    } while ( Graph.NodePool.IsValidIndex( search_node_index ) && Graph.NodePool[ search_node_index ].NodeRef != Parameters.StartLink && ensure( path_length < FGraphAStarDefaultPolicy::FatalPathLength ) );

    if ( path_length >= FGraphAStarDefaultPolicy::FatalPathLength )
    {
        return false;
    }

    // Same as FGraphAStar except we add the start link as the first node, since it is different from where the start location is
    link_path.Reset( path_length + 1 );
    link_path.AddZeroed( path_length + 1 );

    search_node_index = BestNodeIndex;
    int32 result_node_index = path_length;
    do
    {
        link_path[ result_node_index-- ] = Graph.NodePool[ search_node_index ].NodeRef;
        search_node_index = Graph.NodePool[ search_node_index ].ParentNodeIndex;
    } while ( result_node_index >= 0 && search_node_index != INDEX_NONE );

    link_path[ 0 ] = Parameters.StartLink;

    return true;
}

ESVOPathFindingAlgorithmStepperStatus FSVOPathFindingAlgorithmStepper_AStar::Init( EGraphAStarResult & result )
{
    if ( !( Graph.Graph.IsValidRef( Parameters.StartLink ) && Graph.Graph.IsValidRef( Parameters.EndLink ) ) )
    {
        result = SearchFail;
        return ESVOPathFindingAlgorithmStepperStatus::IsStopped;
    }

    if ( Parameters.StartLink == Parameters.EndLink )
    {
        result = SearchSuccess;
        return ESVOPathFindingAlgorithmStepperStatus::IsStopped;
    }

    if ( FGraphAStarDefaultPolicy::bReuseNodePoolInSubsequentSearches )
    {
        Graph.NodePool.ReinitNodes();
    }
    else
    {
        Graph.NodePool.Reset();
    }
    Graph.OpenList.Reset();

    // kick off the search with the first node
    auto & start_node = Graph.NodePool.Add( FSVOGraphAStar::FSearchNode( Parameters.StartLink ) );
    start_node.ParentRef.Invalidate();
    start_node.TraversalCost = 0;
    start_node.TotalCost = GetHeuristicCost( Parameters.StartLink, Parameters.EndLink );

    Graph.OpenList.Push( start_node );

    BestNodeIndex = start_node.SearchNodeIndex;
    BestNodeCost = start_node.TotalCost;

    if ( Graph.OpenList.Num() == 0 )
    {
        SetState( ESVOPathFindingAlgorithmState::Ended );
    }
    else
    {
        SetState( ESVOPathFindingAlgorithmState::ProcessNode );
    }

    return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
}

void FSVOPathFindingAlgorithmStepper_AStar::FillLinkNeighbors( const FSVOOctreeLink & link )
{
    Neighbors.Reset();
    Graph.Graph.GetNeighbors( Neighbors, link );
    NeighborIndex = 0;
}

float FSVOPathFindingAlgorithmStepper_AStar::AdjustTotalCostWithNodeSizeCompensation( const float total_cost, const FSVOOctreeLink neighbor_link ) const
{
    if ( !Parameters.QueryFilterSettings.bUseNodeSizeCompensation )
    {
        return total_cost;
    }

    return total_cost * Parameters.BoundsNavigationData->GetLayerInverseRatio( neighbor_link.LayerIndex );
}

ESVOPathFindingAlgorithmStepperStatus FSVOPathFindingAlgorithmStepper_AStar::ProcessSingleNode( EGraphAStarResult & result )
{
    ConsideredNodeIndex = Graph.OpenList.PopIndex();
    auto & considered_node_unsafe = Graph.NodePool[ ConsideredNodeIndex ];
    considered_node_unsafe.MarkClosed();

    if ( considered_node_unsafe.NodeRef == Parameters.EndLink )
    {
        BestNodeIndex = considered_node_unsafe.SearchNodeIndex;
        BestNodeCost = 0.0f;
        State = ESVOPathFindingAlgorithmState::Ended;
        result = SearchSuccess;
    }
    else
    {
        FillLinkNeighbors( considered_node_unsafe.NodeRef );

        State = ESVOPathFindingAlgorithmState::ProcessNeighbor;

        for ( const auto observer : Observers )
        {
            observer->OnProcessSingleNode( considered_node_unsafe );
        }
    }

    return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
}

ESVOPathFindingAlgorithmStepperStatus FSVOPathFindingAlgorithmStepper_AStar::ProcessNeighbor()
{
    NeighborIndexIncrement neighbor_index_increment( Neighbors, NeighborIndex, State );

    const auto neighbor_link = Neighbors[ NeighborIndex ];

    if ( !Graph.Graph.IsValidRef( neighbor_link ) || neighbor_link == Graph.NodePool[ ConsideredNodeIndex ].ParentRef || neighbor_link == Graph.NodePool[ ConsideredNodeIndex ].NodeRef )
    {
        return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
    }

    auto & neighbor_node = Graph.NodePool.FindOrAdd( neighbor_link );

    if ( neighbor_node.bIsClosed )
    {
        return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
    }

    const auto new_traversal_cost = GetTraversalCost( Graph.NodePool[ ConsideredNodeIndex ].NodeRef, neighbor_node.NodeRef ) + Graph.NodePool[ ConsideredNodeIndex ].TraversalCost;
    const auto new_heuristic_cost = neighbor_node.NodeRef != Parameters.EndLink
                                        ? GetHeuristicCost( neighbor_node.NodeRef, Parameters.EndLink )
                                        : 0.f;
    const auto new_total_cost = AdjustTotalCostWithNodeSizeCompensation( new_traversal_cost + new_heuristic_cost, neighbor_link );

    auto & considered_node_unsafe = Graph.NodePool[ ConsideredNodeIndex ];

    if ( new_total_cost >= neighbor_node.TotalCost )
    {
        for ( const auto observer : Observers )
        {
            observer->OnProcessNeighbor( considered_node_unsafe, neighbor_link, new_total_cost );
        }

        return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
    }

    neighbor_node.TraversalCost = new_traversal_cost;
    ensure( new_traversal_cost > 0 );
    neighbor_node.TotalCost = new_total_cost;
    neighbor_node.ParentRef = Graph.NodePool[ ConsideredNodeIndex ].NodeRef;
    neighbor_node.ParentNodeIndex = Graph.NodePool[ ConsideredNodeIndex ].SearchNodeIndex;
    neighbor_node.MarkNotClosed();

    if ( neighbor_node.IsOpened() == false )
    {
        Graph.OpenList.Push( neighbor_node );
    }

    for ( const auto observer : Observers )
    {
        observer->OnProcessNeighbor( neighbor_node );
    }

    if ( new_heuristic_cost < BestNodeCost )
    {
        BestNodeCost = new_heuristic_cost;
        BestNodeIndex = neighbor_node.SearchNodeIndex;
    }

    return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
}

ESVOPathFindingAlgorithmStepperStatus FSVOPathFindingAlgorithmStepper_AStar::Ended( EGraphAStarResult & result )
{
    if ( BestNodeCost != 0.f )
    {
        result = EGraphAStarResult::GoalUnreachable;
    }

    if ( result == EGraphAStarResult::SearchSuccess )
    {
        TArray< FSVOOctreeLink > link_path;

        if ( !FillLinkPath( link_path ) )
        {
            result = EGraphAStarResult::InfiniteLoop;
        }

        for ( const auto & observer : Observers )
        {
            observer->OnSearchSuccess( link_path );
        }
    }

    return ESVOPathFindingAlgorithmStepperStatus::IsStopped;
}

FSVOPathFindingAlgorithmStepper_AStar::NeighborIndexIncrement::NeighborIndexIncrement( TArray< FSVOOctreeLink > & neighbors, int & neighbor_index, ESVOPathFindingAlgorithmState & state ) :
    Neighbors( neighbors ),
    NeighborIndex( neighbor_index ),
    State( state )
{
}

FSVOPathFindingAlgorithmStepper_AStar::NeighborIndexIncrement::~NeighborIndexIncrement()
{
    NeighborIndex++;

    if ( NeighborIndex >= Neighbors.Num() )
    {
        State = ESVOPathFindingAlgorithmState::ProcessNode;
    }
    else
    {
        State = ESVOPathFindingAlgorithmState::ProcessNeighbor;
    }
}

FSVOPathFindingAlgorithmStepper_ThetaStar::FSVOPathFindingAlgorithmStepper_ThetaStar( const FSVOPathFindingParameters & parameters, const FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters & theta_star_parameters ) :
    FSVOPathFindingAlgorithmStepper_AStar( parameters ),
    ThetaStarParameters( theta_star_parameters ),
    LOSCheckCount( 0 )
{
}

ESVOPathFindingAlgorithmStepperStatus FSVOPathFindingAlgorithmStepper_ThetaStar::Init( EGraphAStarResult & result )
{
    LOSCheckCount = 0;
    return FSVOPathFindingAlgorithmStepper_AStar::Init( result );
}

// Pretty much the same as A* except we try to check if there's a line of sight between the parent of the current node and the neighbor, and path from that parent to the neighbor if there is
ESVOPathFindingAlgorithmStepperStatus FSVOPathFindingAlgorithmStepper_ThetaStar::ProcessNeighbor()
{
    NeighborIndexIncrement neighbor_index_increment( Neighbors, NeighborIndex, State );

    const auto neighbor_link = Neighbors[ NeighborIndex ];

    if ( !Graph.Graph.IsValidRef( neighbor_link ) || neighbor_link == Graph.NodePool[ ConsideredNodeIndex ].ParentRef || neighbor_link == Graph.NodePool[ ConsideredNodeIndex ].NodeRef )
    {
        return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
    }

    auto & neighbor_node = Graph.NodePool.FindOrAdd( neighbor_link );

    if ( neighbor_node.bIsClosed )
    {
        return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
    }

    const auto & current_node = Graph.NodePool[ ConsideredNodeIndex ];
    const auto & current_node_link = current_node.NodeRef;
    const auto & parent_node_link = current_node.ParentRef;
    const auto parent_search_node_index = current_node.ParentNodeIndex;

    const auto has_line_of_sight = parent_search_node_index == INDEX_NONE
                                       ? false
                                       : HasLineOfSight( parent_node_link, neighbor_node.NodeRef );

    float new_traversal_cost;
    const auto new_heuristic_cost = neighbor_node.NodeRef != Parameters.EndLink
                                        ? GetHeuristicCost( neighbor_node.NodeRef, Parameters.EndLink )
                                        : 0.f;

    const auto parent_index = parent_search_node_index == INDEX_NONE
                                  ? 0
                                  : parent_search_node_index;

    const auto & parent_node = Graph.NodePool[ parent_index ];

    FSVOOctreeLink neighbor_parent_node_link;
    int32 neighbor_parent_node_index;

    if ( has_line_of_sight )
    {
        new_traversal_cost = parent_node.TraversalCost + GetTraversalCost( parent_node.NodeRef, neighbor_node.NodeRef );
        neighbor_parent_node_link = parent_node.NodeRef;
        neighbor_parent_node_index = parent_node.SearchNodeIndex;
    }
    else
    {
        new_traversal_cost = current_node.TraversalCost + GetTraversalCost( current_node_link, neighbor_node.NodeRef );
        neighbor_parent_node_link = current_node_link;
        neighbor_parent_node_index = current_node.SearchNodeIndex;
    }

    const auto new_total_cost = AdjustTotalCostWithNodeSizeCompensation( new_traversal_cost + new_heuristic_cost, neighbor_link );

    auto & considered_node_unsafe = Graph.NodePool[ ConsideredNodeIndex ];

    if ( new_total_cost >= neighbor_node.TotalCost )
    {
        for ( const auto observer : Observers )
        {
            observer->OnProcessNeighbor( considered_node_unsafe, neighbor_link, new_total_cost );
        }

        return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
    }

    neighbor_node.TraversalCost = new_traversal_cost;
    ensure( new_traversal_cost > 0 );
    neighbor_node.TotalCost = new_total_cost;
    neighbor_node.ParentRef = neighbor_parent_node_link;
    neighbor_node.ParentNodeIndex = neighbor_parent_node_index;
    neighbor_node.MarkNotClosed();

    if ( neighbor_node.IsOpened() == false )
    {
        Graph.OpenList.Push( neighbor_node );
    }

    for ( const auto observer : Observers )
    {
        observer->OnProcessNeighbor( neighbor_node );
    }

    if ( new_heuristic_cost < BestNodeCost )
    {
        BestNodeCost = new_heuristic_cost;
        BestNodeIndex = neighbor_node.SearchNodeIndex;
    }

    return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
}

ESVOPathFindingAlgorithmStepperStatus FSVOPathFindingAlgorithmStepper_ThetaStar::Ended( EGraphAStarResult & result )
{
    UE_LOG( LogTemp, Verbose, TEXT( "LOSCheckCount : %i" ), LOSCheckCount );
    return FSVOPathFindingAlgorithmStepper_AStar::Ended( result );
}

bool FSVOPathFindingAlgorithmStepper_ThetaStar::HasLineOfSight( const FSVOOctreeLink from, const FSVOOctreeLink to )
{
    auto * world = Parameters.NavigationData.GetWorld();

    if ( !ensure( world != nullptr ) )
    {
        return false;
    }

    const auto from_position = Parameters.BoundsNavigationData->GetLinkPosition( from );
    const auto to_position = Parameters.BoundsNavigationData->GetLinkPosition( to );

    FHitResult hit_result;

    LOSCheckCount++;

    return !UKismetSystemLibrary::SphereTraceSingle(
        world,
        from_position,
        to_position,
        Parameters.AgentProperties.AgentRadius * ThetaStarParameters.AgentRadiusMultiplier,
        //UEngineTypes::ConvertToTraceType( Params.BoundsNavigationData->GetDataGenerationSettings().GenerationSettings.CollisionChannel ),
        ThetaStarParameters.TraceType,
        false,
        TArray< AActor * >(),
        ThetaStarParameters.bShowLineOfSightTraces ? EDrawDebugTrace::ForDuration : EDrawDebugTrace::None,
        hit_result,
        false,
        FLinearColor::Red,
        FLinearColor::Green,
        0.1f );
}

FSVOPathFindingAlgorithmStepper_LazyThetaStar::FSVOPathFindingAlgorithmStepper_LazyThetaStar( const FSVOPathFindingParameters & parameters, const FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters & theta_star_parameters ) :
    FSVOPathFindingAlgorithmStepper_ThetaStar( parameters, theta_star_parameters )
{
}

ESVOPathFindingAlgorithmStepperStatus FSVOPathFindingAlgorithmStepper_LazyThetaStar::ProcessSingleNode( EGraphAStarResult & result )
{
    ConsideredNodeIndex = Graph.OpenList.PopIndex();

    // Take a pointer and not a ref because we may re-assign the current node below if there's no LOS and we add a neighbor to the node pool
    auto * considered_node_unsafe = &Graph.NodePool[ ConsideredNodeIndex ];
    considered_node_unsafe->MarkClosed();

    FillLinkNeighbors( considered_node_unsafe->NodeRef );

    const auto heuristic_scale = Parameters.NavigationQueryFilter.GetHeuristicScale();

    if ( considered_node_unsafe->ParentNodeIndex != INDEX_NONE && !HasLineOfSight( considered_node_unsafe->ParentRef, considered_node_unsafe->NodeRef ) )
    {
        auto min_traversal_cost = TNumericLimits< float >::Max();
        for ( auto neighbor_index = 0; neighbor_index < Neighbors.Num(); ++neighbor_index )
        {
            const auto neighbor_link = Neighbors[ neighbor_index ];
            auto & neighbor_node = Graph.NodePool.FindOrAdd( neighbor_link );

            // FindOrAdd may have re-allocated the node pool. Let's get our pointer back
            considered_node_unsafe = &Graph.NodePool[ ConsideredNodeIndex ];

            if ( !neighbor_node.bIsClosed )
            {
                continue;
            }

            const auto traversal_cost = neighbor_node.TraversalCost + GetTraversalCost( neighbor_node.NodeRef, considered_node_unsafe->NodeRef );
            const float heuristic_cost = neighbor_node.NodeRef != Parameters.EndLink
                                             ? GetHeuristicCost( neighbor_node.NodeRef, Parameters.EndLink )
                                             : 0.f;
            if ( min_traversal_cost > traversal_cost )
            {
                min_traversal_cost = traversal_cost;
                considered_node_unsafe->TraversalCost = traversal_cost;
                considered_node_unsafe->TotalCost = traversal_cost + heuristic_cost;
                considered_node_unsafe->ParentRef = neighbor_link;
                considered_node_unsafe->ParentNodeIndex = neighbor_node.SearchNodeIndex;
            }
        }
    }

    if ( considered_node_unsafe->NodeRef == Parameters.EndLink )
    {
        BestNodeIndex = considered_node_unsafe->SearchNodeIndex;
        BestNodeCost = 0.0f;
        State = ESVOPathFindingAlgorithmState::Ended;
        result = SearchSuccess;
    }
    else
    {
        NeighborIndex = 0;

        State = ESVOPathFindingAlgorithmState::ProcessNeighbor;

        for ( const auto observer : Observers )
        {
            observer->OnProcessSingleNode( *considered_node_unsafe );
        }
    }

    return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
}

ESVOPathFindingAlgorithmStepperStatus FSVOPathFindingAlgorithmStepper_LazyThetaStar::ProcessNeighbor()
{
    NeighborIndexIncrement neighbor_index_increment( Neighbors, NeighborIndex, State );

    const auto neighbor_link = Neighbors[ NeighborIndex ];

    // Again, let's take a pointer as we call FindOrAdd below
    auto * current_node = &Graph.NodePool[ ConsideredNodeIndex ];

    if ( !Graph.Graph.IsValidRef( neighbor_link ) || neighbor_link == current_node->ParentRef || neighbor_link == current_node->NodeRef )
    {
        return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
    }

    auto & neighbor_node = Graph.NodePool.FindOrAdd( neighbor_link );

    if ( neighbor_node.bIsClosed )
    {
        return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
    }

    // Time to refresh
    current_node = &Graph.NodePool[ ConsideredNodeIndex ];

    float new_traversal_cost;
    const auto new_heuristic_cost = neighbor_node.NodeRef != Parameters.EndLink
                                        ? GetHeuristicCost( neighbor_node.NodeRef, Parameters.EndLink )
                                        : 0.f;

    const auto current_node_search_index = current_node->SearchNodeIndex;
    const auto & current_node_link = current_node->NodeRef;
    const auto parent_node_index = current_node->ParentNodeIndex;
    const auto parent_index = parent_node_index == INDEX_NONE ? 0 : parent_node_index;
    const auto & parent_node = Graph.NodePool[ parent_index ];

    FSVOOctreeLink neighbor_parent_link;
    int32 neighbor_parent_search_node_index;

    if ( parent_node_index != INDEX_NONE )
    {
        new_traversal_cost = parent_node.TraversalCost + GetTraversalCost( parent_node.NodeRef, neighbor_node.NodeRef );
        neighbor_parent_link = parent_node.NodeRef;
        neighbor_parent_search_node_index = parent_node.SearchNodeIndex;
    }
    else
    {
        new_traversal_cost = current_node->TraversalCost + GetTraversalCost( current_node_link, neighbor_node.NodeRef );
        neighbor_parent_link = current_node_link;
        neighbor_parent_search_node_index = current_node_search_index;
    }

    const auto new_total_cost = AdjustTotalCostWithNodeSizeCompensation( new_traversal_cost + new_heuristic_cost, neighbor_link );

    if ( new_total_cost >= neighbor_node.TotalCost )
    {
        for ( const auto observer : Observers )
        {
            observer->OnProcessNeighbor( *current_node, neighbor_link, new_total_cost );
        }

        return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
    }

    neighbor_node.TraversalCost = new_traversal_cost;
    ensure( new_traversal_cost > 0 );
    neighbor_node.TotalCost = new_total_cost;
    neighbor_node.ParentRef = neighbor_parent_link;
    neighbor_node.ParentNodeIndex = neighbor_parent_search_node_index;
    neighbor_node.MarkNotClosed();

    if ( neighbor_node.IsOpened() == false )
    {
        Graph.OpenList.Push( neighbor_node );
    }

    for ( const auto observer : Observers )
    {
        observer->OnProcessNeighbor( neighbor_node );
    }

    if ( new_heuristic_cost < BestNodeCost )
    {
        BestNodeCost = new_heuristic_cost;
        BestNodeIndex = neighbor_node.SearchNodeIndex;
    }

    return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
}

ENavigationQueryResult::Type USVOPathFindingAlgorithm::GetPath( FNavigationPath & /*navigation_path*/, const FSVOPathFindingParameters & /*params*/ ) const
{
    return ENavigationQueryResult::Error;
}

TSharedPtr< FSVOPathFindingAlgorithmStepper > USVOPathFindingAlgorithm::GetDebugPathStepper( FSVOPathFinderDebugInfos & /*debug_infos*/, const FSVOPathFindingParameters /*params*/ ) const
{
    return nullptr;
}

ENavigationQueryResult::Type USVOPathFindingAlgorithmAStar::GetPath( FNavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const
{
    FSVOPathFindingAlgorithmStepper_AStar stepper( params );
    const auto path_builder = MakeShared< FSVOPathFindingAStarObserver_BuildPath >( navigation_path, stepper );

    stepper.AddObserver( path_builder );

    int iterations = 0;

    EGraphAStarResult result = EGraphAStarResult::SearchFail;
    while ( stepper.Step( result ) == ESVOPathFindingAlgorithmStepperStatus::MustContinue )
    {
        iterations++;
    }

    return GraphAStarResultToNavigationTypeResult( result );
}

TSharedPtr< FSVOPathFindingAlgorithmStepper > USVOPathFindingAlgorithmAStar::GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const
{
    auto stepper = MakeShared< FSVOPathFindingAlgorithmStepper_AStar >( params );
    const auto debug_path = MakeShared< FSVOPathFindingAStarObserver_GenerateDebugInfos >( debug_infos, stepper.Get() );
    stepper->AddObserver( debug_path );

    return stepper;
}

ENavigationQueryResult::Type USVOPathFindingAlgorithmThetaStar::GetPath( FNavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const
{
    FSVOPathFindingAlgorithmStepper_ThetaStar stepper( params, ThetaStarParameters );
    const auto path_builder = MakeShared< FSVOPathFindingAStarObserver_BuildPath >( navigation_path, stepper );

    stepper.AddObserver( path_builder );

    int iterations = 0;

    EGraphAStarResult result = EGraphAStarResult::SearchFail;
    while ( stepper.Step( result ) == ESVOPathFindingAlgorithmStepperStatus::MustContinue )
    {
        iterations++;
    }

    return GraphAStarResultToNavigationTypeResult( result );
}

TSharedPtr< FSVOPathFindingAlgorithmStepper > USVOPathFindingAlgorithmThetaStar::GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const
{
    auto stepper = MakeShared< FSVOPathFindingAlgorithmStepper_ThetaStar >( params, ThetaStarParameters );
    const auto debug_path = MakeShared< FSVOPathFindingAStarObserver_GenerateDebugInfos >( debug_infos, stepper.Get() );
    stepper->AddObserver( debug_path );

    return stepper;
}

ENavigationQueryResult::Type USVOPathFindingAlgorithmLazyThetaStar::GetPath( FNavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const
{
    FSVOPathFindingAlgorithmStepper_LazyThetaStar stepper( params, ThetaStarParameters );
    const auto path_builder = MakeShared< FSVOPathFindingAStarObserver_BuildPath >( navigation_path, stepper );

    stepper.AddObserver( path_builder );

    int iterations = 0;

    EGraphAStarResult result = EGraphAStarResult::SearchFail;
    while ( stepper.Step( result ) == ESVOPathFindingAlgorithmStepperStatus::MustContinue )
    {
        iterations++;
    }

    return GraphAStarResultToNavigationTypeResult( result );
}

TSharedPtr< FSVOPathFindingAlgorithmStepper > USVOPathFindingAlgorithmLazyThetaStar::GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const
{
    auto stepper = MakeShared< FSVOPathFindingAlgorithmStepper_LazyThetaStar >( params, ThetaStarParameters );
    const auto debug_path = MakeShared< FSVOPathFindingAStarObserver_GenerateDebugInfos >( debug_infos, stepper.Get() );
    stepper->AddObserver( debug_path );

    return stepper;
}

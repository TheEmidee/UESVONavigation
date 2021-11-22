#include "PathFinding/SVOPathFindingAlgorithm_AStar.h"

#include "SVOHelpers.h"
#include "SVOVolumeNavigationData.h"

FSVOPathFindingAlgorithmStepper_AStar::FSVOPathFindingAlgorithmStepper_AStar( const FSVOPathFindingParameters & parameters ) :
    FSVOPathFindingAlgorithmStepper( parameters ),
    ConsideredNodeIndex( INDEX_NONE ),
    BestNodeIndex( INDEX_NONE ),
    BestNodeCost( -1.0f ),
    NeighborIndex( INDEX_NONE )
{
}

bool FSVOPathFindingAlgorithmStepper_AStar::FillNodeAddresses( TArray< FSVONodeAddress > & node_addresses ) const
{
    int32 search_node_index = BestNodeIndex;
    int32 path_length = 0;
    do
    {
        path_length++;
        search_node_index = Graph.NodePool[ search_node_index ].ParentNodeIndex;
    } while ( Graph.NodePool.IsValidIndex( search_node_index ) && Graph.NodePool[ search_node_index ].NodeRef != Parameters.StartNodeAddress && ensure( path_length < FGraphAStarDefaultPolicy::FatalPathLength ) );

    if ( path_length >= FGraphAStarDefaultPolicy::FatalPathLength )
    {
        return false;
    }

    // Same as FGraphAStar except we add the start node address as the first node, since it is different from where the start location is
    node_addresses.Reset( path_length + 1 );
    node_addresses.AddZeroed( path_length + 1 );

    search_node_index = BestNodeIndex;
    int32 result_node_index = path_length;
    do
    {
        node_addresses[ result_node_index-- ] = Graph.NodePool[ search_node_index ].NodeRef;
        search_node_index = Graph.NodePool[ search_node_index ].ParentNodeIndex;
    } while ( result_node_index >= 0 && search_node_index != INDEX_NONE );

    node_addresses[ 0 ] = Parameters.StartNodeAddress;

    return true;
}

ESVOPathFindingAlgorithmStepperStatus FSVOPathFindingAlgorithmStepper_AStar::Init( EGraphAStarResult & result )
{
    if ( !( Graph.Graph.IsValidRef( Parameters.StartNodeAddress ) && Graph.Graph.IsValidRef( Parameters.EndNodeAddress ) ) )
    {
        result = SearchFail;
        return ESVOPathFindingAlgorithmStepperStatus::IsStopped;
    }

    if ( Parameters.StartNodeAddress == Parameters.EndNodeAddress )
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
    auto & start_node = Graph.NodePool.Add( FSVOGraphAStar::FSearchNode( Parameters.StartNodeAddress ) );
    start_node.ParentRef.Invalidate();
    start_node.TraversalCost = 0;
    start_node.TotalCost = GetHeuristicCost( Parameters.StartNodeAddress, Parameters.EndNodeAddress );

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

void FSVOPathFindingAlgorithmStepper_AStar::FillNodeAddressNeighbors( const FSVONodeAddress & node_address )
{
    Neighbors.Reset();
    Graph.Graph.GetNodeNeighbors( Neighbors, node_address );
    NeighborIndex = 0;
}

float FSVOPathFindingAlgorithmStepper_AStar::AdjustTotalCostWithNodeSizeCompensation( const float total_cost, const FSVONodeAddress neighbor_node_address ) const
{
    if ( !Parameters.QueryFilterSettings.bUseNodeSizeCompensation )
    {
        return total_cost;
    }

    return total_cost * Parameters.VolumeNavigationData->GetLayerInverseRatio( neighbor_node_address.LayerIndex );
}

ESVOPathFindingAlgorithmStepperStatus FSVOPathFindingAlgorithmStepper_AStar::ProcessSingleNode( EGraphAStarResult & result )
{
    ConsideredNodeIndex = Graph.OpenList.PopIndex();
    auto & considered_node_unsafe = Graph.NodePool[ ConsideredNodeIndex ];
    considered_node_unsafe.MarkClosed();

    if ( considered_node_unsafe.NodeRef == Parameters.EndNodeAddress )
    {
        BestNodeIndex = considered_node_unsafe.SearchNodeIndex;
        BestNodeCost = 0.0f;
        State = ESVOPathFindingAlgorithmState::Ended;
        result = SearchSuccess;
    }
    else
    {
        FillNodeAddressNeighbors( considered_node_unsafe.NodeRef );

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

    const auto neighbor_address = Neighbors[ NeighborIndex ];

    if ( !Graph.Graph.IsValidRef( neighbor_address ) || neighbor_address == Graph.NodePool[ ConsideredNodeIndex ].ParentRef || neighbor_address == Graph.NodePool[ ConsideredNodeIndex ].NodeRef )
    {
        return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
    }

    auto & neighbor_node = Graph.NodePool.FindOrAdd( neighbor_address );

    if ( neighbor_node.bIsClosed )
    {
        return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
    }

    const auto new_traversal_cost = GetTraversalCost( Graph.NodePool[ ConsideredNodeIndex ].NodeRef, neighbor_node.NodeRef ) + Graph.NodePool[ ConsideredNodeIndex ].TraversalCost;
    const auto new_heuristic_cost = neighbor_node.NodeRef != Parameters.EndNodeAddress
                                        ? GetHeuristicCost( neighbor_node.NodeRef, Parameters.EndNodeAddress )
                                        : 0.f;
    const auto new_total_cost = AdjustTotalCostWithNodeSizeCompensation( new_traversal_cost + new_heuristic_cost, neighbor_address );

    const auto & considered_node_unsafe = Graph.NodePool[ ConsideredNodeIndex ];

    if ( new_total_cost >= neighbor_node.TotalCost )
    {
        for ( const auto observer : Observers )
        {
            observer->OnProcessNeighbor( considered_node_unsafe, neighbor_address, new_total_cost );
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
        TArray< FSVONodeAddress > node_addresses;

        if ( !FillNodeAddresses( node_addresses ) )
        {
            result = EGraphAStarResult::InfiniteLoop;
        }

        for ( const auto & observer : Observers )
        {
            observer->OnSearchSuccess( node_addresses );
        }
    }

    return ESVOPathFindingAlgorithmStepperStatus::IsStopped;
}

FSVOPathFindingAlgorithmStepper_AStar::NeighborIndexIncrement::NeighborIndexIncrement( TArray< FSVONodeAddress > & neighbors, int & neighbor_index, ESVOPathFindingAlgorithmState & state ) :
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

    return FSVOHelpers::GraphAStarResultToNavigationTypeResult( result );
}

TSharedPtr< FSVOPathFindingAlgorithmStepper > USVOPathFindingAlgorithmAStar::GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const
{
    auto stepper = MakeShared< FSVOPathFindingAlgorithmStepper_AStar >( params );
    const auto debug_path = MakeShared< FSVOPathFindingAStarObserver_GenerateDebugInfos >( debug_infos, stepper.Get() );
    stepper->AddObserver( debug_path );

    return stepper;
}
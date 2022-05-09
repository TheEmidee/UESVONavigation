#include "PathFinding/SVOPathFindingAlgorithm_LazyThetaStar.h"

#include "PathFinding/SVOPathFindingAlgorithm.h"
#include "SVOHelpers.h"

FSVOPathFindingAlgorithmStepper_LazyThetaStar::FSVOPathFindingAlgorithmStepper_LazyThetaStar( const FSVOPathFindingParameters & parameters, const FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters & theta_star_parameters ) :
    FSVOPathFindingAlgorithmStepper_ThetaStar( parameters, theta_star_parameters )
{
}

ESVOPathFindingAlgorithmStepperStatus FSVOPathFindingAlgorithmStepper_LazyThetaStar::ProcessSingleNode( EGraphAStarResult & result )
{
    if ( Graph.OpenList.Num() == 0 )
    {
        State = ESVOPathFindingAlgorithmState::Ended;
        result = SearchFail;
        return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
    }

    ConsideredNodeIndex = Graph.OpenList.PopIndex();

    // Take a pointer and not a ref because we may re-assign the current node below if there's no LOS and we add a neighbor to the node pool
    auto * considered_node_unsafe = &Graph.NodePool[ ConsideredNodeIndex ];
    considered_node_unsafe->MarkClosed();

    FillNodeAddressNeighbors( considered_node_unsafe->NodeRef );

    const auto heuristic_scale = Parameters.NavigationQueryFilter.GetHeuristicScale();

    if ( considered_node_unsafe->ParentNodeIndex != INDEX_NONE && !HasLineOfSight( considered_node_unsafe->ParentRef, considered_node_unsafe->NodeRef ) )
    {
        auto min_traversal_cost = TNumericLimits< float >::Max();
        for ( auto neighbor_index = 0; neighbor_index < Neighbors.Num(); ++neighbor_index )
        {
            const auto neighbor_address = Neighbors[ neighbor_index ];
            auto & neighbor_node = Graph.NodePool.FindOrAdd( neighbor_address );

            // FindOrAdd may have re-allocated the node pool. Let's get our pointer back
            considered_node_unsafe = &Graph.NodePool[ ConsideredNodeIndex ];

            if ( !neighbor_node.bIsClosed )
            {
                continue;
            }

            const auto traversal_cost = neighbor_node.TraversalCost + GetTraversalCost( neighbor_node.NodeRef, considered_node_unsafe->NodeRef );
            const float heuristic_cost = neighbor_node.NodeRef != Parameters.EndNodeAddress
                                             ? GetHeuristicCost( neighbor_node.NodeRef, Parameters.EndNodeAddress )
                                             : 0.f;
            if ( min_traversal_cost > traversal_cost )
            {
                min_traversal_cost = traversal_cost;
                considered_node_unsafe->TraversalCost = traversal_cost;
                considered_node_unsafe->TotalCost = traversal_cost + heuristic_cost;
                considered_node_unsafe->ParentRef = neighbor_address;
                considered_node_unsafe->ParentNodeIndex = neighbor_node.SearchNodeIndex;
            }
        }
    }

    if ( considered_node_unsafe->NodeRef == Parameters.EndNodeAddress )
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

ESVOPathFindingAlgorithmStepperStatus FSVOPathFindingAlgorithmStepper_LazyThetaStar::ProcessNeighbor( EGraphAStarResult & result )
{
    NeighborIndexIncrement neighbor_index_increment( Neighbors, NeighborIndex, State );

    const auto neighbor_address = Neighbors[ NeighborIndex ];

    // Again, let's take a pointer as we call FindOrAdd below
    auto * current_node = &Graph.NodePool[ ConsideredNodeIndex ];

    if ( !Graph.Graph.IsValidRef( neighbor_address ) || neighbor_address == current_node->ParentRef || neighbor_address == current_node->NodeRef )
    {
        return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
    }

    auto & neighbor_node = Graph.NodePool.FindOrAdd( neighbor_address );

    if ( neighbor_node.bIsClosed )
    {
        return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
    }

    // Time to refresh
    current_node = &Graph.NodePool[ ConsideredNodeIndex ];

    float new_traversal_cost;
    const auto new_heuristic_cost = neighbor_node.NodeRef != Parameters.EndNodeAddress
                                        ? GetHeuristicCost( neighbor_node.NodeRef, Parameters.EndNodeAddress )
                                        : 0.f;

    const auto current_node_search_index = current_node->SearchNodeIndex;
    const auto & current_node_address = current_node->NodeRef;
    const auto parent_node_index = current_node->ParentNodeIndex;
    const auto parent_index = parent_node_index == INDEX_NONE ? 0 : parent_node_index;
    const auto & parent_node = Graph.NodePool[ parent_index ];

    FSVONodeAddress neighbor_parent_address;
    int32 neighbor_parent_search_node_index;

    if ( parent_node_index != INDEX_NONE )
    {
        new_traversal_cost = parent_node.TraversalCost + GetTraversalCost( parent_node.NodeRef, neighbor_node.NodeRef );
        neighbor_parent_address = parent_node.NodeRef;
        neighbor_parent_search_node_index = parent_node.SearchNodeIndex;
    }
    else
    {
        new_traversal_cost = current_node->TraversalCost + GetTraversalCost( current_node_address, neighbor_node.NodeRef );
        neighbor_parent_address = current_node_address;
        neighbor_parent_search_node_index = current_node_search_index;
    }

    const auto new_total_cost = AdjustTotalCostWithNodeSizeCompensation( new_traversal_cost + new_heuristic_cost, neighbor_address );

    if ( new_total_cost >= neighbor_node.TotalCost )
    {
        for ( const auto observer : Observers )
        {
            observer->OnProcessNeighbor( *current_node, neighbor_address, new_total_cost );
        }

        return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
    }

    neighbor_node.TraversalCost = new_traversal_cost;
    ensure( new_traversal_cost > 0 );
    neighbor_node.TotalCost = new_total_cost;
    neighbor_node.ParentRef = neighbor_parent_address;
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

ENavigationQueryResult::Type USVOPathFindingAlgorithmLazyThetaStar::GetPath( FSVONavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const
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

    return FSVOHelpers::GraphAStarResultToNavigationTypeResult( result );
}

TSharedPtr< FSVOPathFindingAlgorithmStepper > USVOPathFindingAlgorithmLazyThetaStar::GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const
{
    auto stepper = MakeShared< FSVOPathFindingAlgorithmStepper_LazyThetaStar >( params, ThetaStarParameters );
    const auto debug_path = MakeShared< FSVOPathFindingAStarObserver_GenerateDebugInfos >( debug_infos, stepper.Get() );
    stepper->AddObserver( debug_path );

    return stepper;
}

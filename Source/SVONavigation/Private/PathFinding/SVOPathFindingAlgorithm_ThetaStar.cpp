#include "PathFinding/SVOPathFindingAlgorithm_ThetaStar.h"

#include "Raycasters/SVORaycaster_OctreeTraversal.h"
#include "SVOHelpers.h"
#include "SVONavigationData.h"
#include "SVONavigationSettings.h"

FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters::FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters() :
    RayCaster( NewObject< USVORayCaster_OctreeTraversal >() )
{
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

    const auto neighbor_node_address = Neighbors[ NeighborIndex ];

    if ( !Graph.Graph.IsValidRef( neighbor_node_address ) || neighbor_node_address == Graph.NodePool[ ConsideredNodeIndex ].ParentRef || neighbor_node_address == Graph.NodePool[ ConsideredNodeIndex ].NodeRef )
    {
        return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
    }

    auto & neighbor_node = Graph.NodePool.FindOrAdd( neighbor_node_address );

    if ( neighbor_node.bIsClosed )
    {
        return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
    }

    const auto & current_node = Graph.NodePool[ ConsideredNodeIndex ];
    const auto & current_node_address = current_node.NodeRef;
    const auto & parent_node_address = current_node.ParentRef;
    const auto parent_search_node_index = current_node.ParentNodeIndex;

    const auto has_line_of_sight = parent_search_node_index == INDEX_NONE
                                       ? false
                                       : HasLineOfSight( parent_node_address, neighbor_node.NodeRef );

    float new_traversal_cost;
    const auto new_heuristic_cost = neighbor_node.NodeRef != Parameters.EndNodeAddress
                                        ? GetHeuristicCost( neighbor_node.NodeRef, Parameters.EndNodeAddress )
                                        : 0.f;

    const auto parent_index = parent_search_node_index == INDEX_NONE
                                  ? 0
                                  : parent_search_node_index;

    const auto & parent_node = Graph.NodePool[ parent_index ];

    FSVONodeAddress neighbor_parent_node_address;
    int32 neighbor_parent_node_index;

    if ( has_line_of_sight )
    {
        new_traversal_cost = parent_node.TraversalCost + GetTraversalCost( parent_node.NodeRef, neighbor_node.NodeRef );
        neighbor_parent_node_address = parent_node.NodeRef;
        neighbor_parent_node_index = parent_node.SearchNodeIndex;
    }
    else
    {
        new_traversal_cost = current_node.TraversalCost + GetTraversalCost( current_node_address, neighbor_node.NodeRef );
        neighbor_parent_node_address = current_node_address;
        neighbor_parent_node_index = current_node.SearchNodeIndex;
    }

    const auto new_total_cost = AdjustTotalCostWithNodeSizeCompensation( new_traversal_cost + new_heuristic_cost, neighbor_node_address );

    const auto & considered_node_unsafe = Graph.NodePool[ ConsideredNodeIndex ];

    if ( new_total_cost >= neighbor_node.TotalCost )
    {
        for ( const auto observer : Observers )
        {
            observer->OnProcessNeighbor( considered_node_unsafe, neighbor_node_address, new_total_cost );
        }

        return ESVOPathFindingAlgorithmStepperStatus::MustContinue;
    }

    neighbor_node.TraversalCost = new_traversal_cost;
    ensure( new_traversal_cost > 0 );
    neighbor_node.TotalCost = new_total_cost;
    neighbor_node.ParentRef = neighbor_parent_node_address;
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

bool FSVOPathFindingAlgorithmStepper_ThetaStar::HasLineOfSight( const FSVONodeAddress from, const FSVONodeAddress to ) const
{
    const auto * ray_caster = ThetaStarParameters.RayCaster;

    if ( ray_caster == nullptr )
    {
        ray_caster = GetDefault< USVONavigationSettings >()->DefaultRaycasterClass->GetDefaultObject< USVORayCaster >();
    }

    return !ray_caster->Trace( Parameters.VolumeNavigationData, from, to );
}

ENavigationQueryResult::Type USVOPathFindingAlgorithmThetaStar::GetPath( FSVONavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const
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

    return FSVOHelpers::GraphAStarResultToNavigationTypeResult( result );
}

TSharedPtr< FSVOPathFindingAlgorithmStepper > USVOPathFindingAlgorithmThetaStar::GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const
{
    auto stepper = MakeShared< FSVOPathFindingAlgorithmStepper_ThetaStar >( params, ThetaStarParameters );
    const auto debug_path = MakeShared< FSVOPathFindingAStarObserver_GenerateDebugInfos >( debug_infos, stepper.Get() );
    stepper->AddObserver( debug_path );

    return stepper;
}
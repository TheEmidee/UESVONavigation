#include "PathFinding/SVOPathFindingAlgorithmTypes.h"

#include "PathFinding/SVONavigationQueryFilterImpl.h"
#include "Pathfinding/SVONavigationQueryFilterSettings.h"
#include "SVOVolumeNavigationData.h"

FSVONodeAddressWithLocation::FSVONodeAddressWithLocation( const FSVONodeAddress & node_address, const FVector & location ) :
    NodeAddress( node_address ),
    Location( location )
{}

FSVONodeAddressWithLocation::FSVONodeAddressWithLocation( const FSVONodeAddress & node_address, const FSVOVolumeNavigationData & bounds_navigation_data ) :
    NodeAddress( node_address ),
    Location( bounds_navigation_data.GetNodePositionFromAddress( node_address, true ) )
{}

FSVOPathFinderDebugNodeCost::FSVOPathFinderDebugNodeCost( const FSVONodeAddressWithLocation & from, const FSVONodeAddressWithLocation & to, const float cost, const bool is_closed ) :
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

FSVOPathFinderNodeAddressWithCost::FSVOPathFinderNodeAddressWithCost( const FSVONodeAddress & node_address, const double traversal_cost ) :
    NodeAddress( node_address ),
    Cost( traversal_cost )
{}

FSVOPathFinderDebugInfos::FSVOPathFinderDebugInfos():
    LastProcessedSingleNode(),
    Iterations( 0 ),
    VisitedNodes( 0 ),
    PathSegmentCount( 0 ),
    PathLength( 0 )
{
}

void FSVOPathFinderDebugInfos::Reset()
{
    LastProcessedSingleNode.Reset();
    ProcessedNeighbors.Reset();
    Iterations = 0;
    VisitedNodes = 0;
    CurrentBestPath.ResetForRepath();
}

FSVOPathFindingParameters::FSVOPathFindingParameters( const FSVOVolumeNavigationData & volume_navigation_data, const FVector & start_location, const FVector & end_location, const FNavigationQueryFilter & nav_query_filter ) :
    StartLocation( start_location ),
    EndLocation( end_location ),
    NavigationQueryFilter( nav_query_filter ),
    QueryFilterImplementation( static_cast< const FSVONavigationQueryFilterImpl * >( NavigationQueryFilter.GetImplementation() ) ),
    QueryFilterSettings( QueryFilterImplementation->QueryFilterSettings ),
    HeuristicCalculator( QueryFilterSettings.HeuristicCalculator ),
    CostCalculator( QueryFilterSettings.TraversalCostCalculator ),
    VolumeNavigationData( volume_navigation_data )
{
}

TOptional< FSVOPathFindingParameters > FSVOPathFindingParameters::Initialize( const FSVOVolumeNavigationData & volume_navigation_data, const FVector & start_location, const FVector & end_location, const FNavigationQueryFilter & nav_query_filter )
{
    auto result = FSVOPathFindingParameters( volume_navigation_data, start_location, end_location, nav_query_filter );

    if ( volume_navigation_data.GetNodeAddressFromPosition( result.StartNodeAddress, start_location ) )
    {
        if ( volume_navigation_data.GetNodeAddressFromPosition( result.EndNodeAddress, end_location ) )
        {
            return result;
        }
    }

    return TOptional< FSVOPathFindingParameters >();
}

#include "PathFinding/SVOPathFindingAlgorithmObservers.h"

#include "PathFinding/SVOPathFindingAlgorithm.h"
#include "SVOVolumeNavigationData.h"

#include <NavigationPath.h>

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

    void BuildPath( FNavigationPath & path, const FSVOPathFindingParameters & params, const TArray< FSVONodeAddress > & node_addresses, const bool add_end_location )
    {
        auto & path_points = path.GetPathPoints();
        const auto path_points_size = node_addresses.Num() + 1;

        path_points.Reset( path_points_size );

        path_points.Emplace( params.StartLocation );

        const auto & bounds_data = *params.VolumeNavigationData;

        for ( auto index = 0; index < node_addresses.Num() - 1; index++ )
        {
            const auto address = node_addresses[ index ];
            path_points.Emplace( bounds_data.GetNodePositionFromAddress( address ) );
        }

        if ( add_end_location )
        {
            path_points.Emplace( params.EndLocation );
        }
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

void FSVOPathFindingAStarObserver_BuildPath::OnSearchSuccess( const TArray< FSVONodeAddress > & node_addresses )
{
    const auto & params = Stepper.GetParameters();

    BuildPath( NavigationPath, params, node_addresses, true );

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

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnProcessSingleNode( const FGraphAStarDefaultNode< FSVOVolumeNavigationData > & node )
{
    if ( node.ParentRef.IsValid() )
    {
        DebugInfos.LastProcessedSingleNode.From = FSVONodeAddressWithLocation( node.ParentRef, *Stepper.GetParameters().VolumeNavigationData );
    }
    else
    {
        DebugInfos.LastProcessedSingleNode.From = FSVONodeAddressWithLocation( FSVONodeAddress::InvalidAddress, Stepper.GetParameters().StartLocation );
    }

    DebugInfos.LastProcessedSingleNode.To = FSVONodeAddressWithLocation( node.NodeRef, *Stepper.GetParameters().VolumeNavigationData );
    DebugInfos.LastProcessedSingleNode.Cost = node.TotalCost;

    DebugInfos.ProcessedNeighbors.Reset();

    DebugInfos.Iterations++;

    TArray< FSVONodeAddress > node_addresses;
    Stepper.FillNodeAddresses( node_addresses );

    // Fill DebugInfos.CurrentBestPath
    FillCurrentBestPath( node_addresses, false );
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnProcessNeighbor( const FGraphAStarDefaultNode< FSVOVolumeNavigationData > & parent, const FGraphAStarDefaultNode< FSVOVolumeNavigationData > & neighbor, const float cost )
{
    DebugInfos.ProcessedNeighbors.Emplace( FSVONodeAddressWithLocation( parent.NodeRef, *Stepper.GetParameters().VolumeNavigationData ), FSVONodeAddressWithLocation( neighbor.NodeRef, *Stepper.GetParameters().VolumeNavigationData ), cost, true );
    DebugInfos.VisitedNodes++;
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnProcessNeighbor( const FGraphAStarDefaultNode< FSVOVolumeNavigationData > & neighbor )
{
    DebugInfos.ProcessedNeighbors.Emplace( FSVONodeAddressWithLocation( neighbor.ParentRef, *Stepper.GetParameters().VolumeNavigationData ), FSVONodeAddressWithLocation( neighbor.NodeRef, *Stepper.GetParameters().VolumeNavigationData ), neighbor.TotalCost, false );
    DebugInfos.VisitedNodes++;
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnSearchSuccess( const TArray< FSVONodeAddress > & node_addresses )
{
    FillCurrentBestPath( node_addresses, true );

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

void FSVOPathFindingAStarObserver_GenerateDebugInfos::FillCurrentBestPath( const TArray< FSVONodeAddress > & node_addresses, const bool add_end_location ) const
{
    DebugInfos.CurrentBestPath.ResetForRepath();

    const auto & params = Stepper.GetParameters();

    BuildPath( DebugInfos.CurrentBestPath, params, node_addresses, add_end_location );

    if ( params.VerticalOffset != 0.0f )
    {
        ApplyVerticalOffset( DebugInfos.CurrentBestPath, params.VerticalOffset );
    }

    DebugInfos.CurrentBestPath.MarkReady();
}
#include "PathFinding/SVOPathFindingAlgorithmObservers.h"

#include "PathFinding/SVOPathFindingAlgorithm.h"
#include "Pathfinding/SVONavigationQueryFilterSettings.h"
#include "SVOVolumeNavigationData.h"

#include <NavigationPath.h>

DEFINE_LOG_CATEGORY_STATIC( LogSVODebugPathStepper, Verbose, Verbose )

namespace
{
    void BuildPath( FSVONavigationPath & path, const FSVOPathFindingParameters & params, const TArray< FSVOPathFinderNodeAddressWithCost > & node_addresses, const bool add_end_location )
    {
        auto & path_points = path.GetPathPoints();
        auto & path_point_costs = path.GetPathPointCosts();

        const auto path_points_size = node_addresses.Num() + 1;

        ensureAlways( node_addresses[ 0 ].NodeAddress == params.StartNodeAddress );

        const auto & bounds_data = params.VolumeNavigationData;

        path_points.Reset( path_points_size );
        path_point_costs.Reset( path_points_size );

        path_points.Emplace( params.StartLocation );
        path_point_costs.Add( 0.0f );

        for ( auto index = 1; index < node_addresses.Num() - 1; index++ )
        {
            const auto address_with_cost = node_addresses[ index ];
            path_points.Emplace( bounds_data.GetNodePositionFromAddress( address_with_cost.NodeAddress, true ) );
            path_point_costs.Add( address_with_cost.Cost );
        }

        if ( add_end_location )
        {
            path_points.Emplace( params.EndLocation );
            path_point_costs.Add( node_addresses.Last().Cost );
        }
    }

    struct FSVOCatmullRomPath
    {
        explicit FSVOCatmullRomPath( FSVONavigationPath & path, const int subdivisions )
        {
            auto old_points = path.GetPathPoints();
            auto old_costs = path.GetPathPointCosts();

            auto & path_points = path.GetPathPoints();
            auto & path_point_costs = path.GetPathPointCosts();

            old_points.Insert( 2 * ( old_points[ 0 ].Location - old_points[ 1 ].Location ), 0 );
            old_points.Emplace( 2 * ( old_points.Last().Location - old_points.Last( 1 ).Location ) );

            const auto new_size = ( old_points.Num() - 3 ) * subdivisions;
            path_points.Reset( new_size );
            path_point_costs.Reset( new_size );

            for ( auto index = 1; index < old_points.Num() - 2; ++index )
            {
                for ( auto alpha = 0; alpha < subdivisions; ++alpha )
                {
                    path_points.Emplace(
                        GetPoint(
                            old_points[ index - 1 ],
                            old_points[ index ],
                            old_points[ index + 1 ],
                            old_points[ index + 2 ],
                            static_cast< float >( alpha ) / subdivisions ) );

                    path_point_costs.Add( old_costs[ index - 1 ] / subdivisions );
                }
            }
        }

    private:
        float GetT( float t, float alpha, const FVector & p0, const FVector & p1 ) const
        {
            const auto d = p1 - p0;
            const auto a = d | d; // Dot product
            const auto b = FMath::Pow( a, alpha * .5f );
            return ( b + t );
        }

        FVector GetPoint( const FVector & p0, const FVector & p1, const FVector & p2, const FVector & p3, float t /* between 0 and 1 */, float alpha = .5f /* between 0 and 1 */ ) const
        {
            constexpr auto t0 = 0.0f;
            const auto t1 = GetT( t0, alpha, p0, p1 );
            const auto t2 = GetT( t1, alpha, p1, p2 );
            const auto t3 = GetT( t2, alpha, p2, p3 );
            t = FMath::Lerp( t1, t2, t );
            const auto a1 = ( t1 - t ) / ( t1 - t0 ) * p0 + ( t - t0 ) / ( t1 - t0 ) * p1;
            const auto a2 = ( t2 - t ) / ( t2 - t1 ) * p1 + ( t - t1 ) / ( t2 - t1 ) * p2;
            const auto a3 = ( t3 - t ) / ( t3 - t2 ) * p2 + ( t - t2 ) / ( t3 - t2 ) * p3;
            const auto b1 = ( t2 - t ) / ( t2 - t0 ) * a1 + ( t - t0 ) / ( t2 - t0 ) * a2;
            const auto b2 = ( t3 - t ) / ( t3 - t1 ) * a2 + ( t - t1 ) / ( t3 - t1 ) * a3;
            const auto c = ( t2 - t ) / ( t2 - t1 ) * b1 + ( t - t1 ) / ( t2 - t1 ) * b2;
            return c;
        }
    };

    void SmoothPath( FSVONavigationPath & path, const int subdivisions )
    {
        FSVOCatmullRomPath catmull_rom_path( path, subdivisions );
    }
}

FSVOPathFindingAlgorithmObserver::FSVOPathFindingAlgorithmObserver( const FSVOPathFindingAlgorithmStepper & stepper ) :
    Stepper( stepper )
{
}

FSVOPathFindingAStarObserver_BuildPath::FSVOPathFindingAStarObserver_BuildPath( FSVONavigationPath & navigation_path, const FSVOPathFindingAlgorithmStepper & stepper ) :
    FSVOPathFindingAlgorithmObserver( stepper ),
    NavigationPath( navigation_path )
{
}

void FSVOPathFindingAStarObserver_BuildPath::OnSearchSuccess( const TArray< FSVOPathFinderNodeAddressWithCost > & node_addresses )
{
    const auto & params = Stepper.GetParameters();

    BuildPath( NavigationPath, params, node_addresses, true );

    if ( params.QueryFilterSettings.bSmoothPaths )
    {
        SmoothPath( NavigationPath, params.QueryFilterSettings.SmoothingSubdivisions );
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
        DebugInfos.LastProcessedSingleNode.From = FSVONodeAddressWithLocation( node.ParentRef, Stepper.GetParameters().VolumeNavigationData );
    }
    else
    {
        DebugInfos.LastProcessedSingleNode.From = FSVONodeAddressWithLocation( FSVONodeAddress::InvalidAddress, Stepper.GetParameters().StartLocation );
    }

    DebugInfos.LastProcessedSingleNode.To = FSVONodeAddressWithLocation( node.NodeRef, Stepper.GetParameters().VolumeNavigationData );
    DebugInfos.LastProcessedSingleNode.Cost = node.TotalCost;

    UE_LOG( LogSVODebugPathStepper, Verbose, TEXT( "OnProcessSingleNode From [%s] To [%s] with cost [%s]" ), *node.NodeRef.ToString(), *DebugInfos.LastProcessedSingleNode.To.NodeAddress.ToString(), *FString::SanitizeFloat( node.TotalCost ) );

    DebugInfos.ProcessedNeighbors.Reset();

    DebugInfos.Iterations++;

    TArray< FSVOPathFinderNodeAddressWithCost > node_addresses;
    Stepper.FillNodeAddresses( node_addresses );

    // Fill DebugInfos.CurrentBestPath
    FillCurrentBestPath( node_addresses, false );
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnProcessNeighbor( const FGraphAStarDefaultNode< FSVOVolumeNavigationData > & parent, const FGraphAStarDefaultNode< FSVOVolumeNavigationData > & neighbor, const float cost )
{
    DebugInfos.ProcessedNeighbors.Emplace( FSVONodeAddressWithLocation( parent.NodeRef, Stepper.GetParameters().VolumeNavigationData ), FSVONodeAddressWithLocation( neighbor.NodeRef, Stepper.GetParameters().VolumeNavigationData ), cost, true );
    DebugInfos.VisitedNodes++;

    UE_LOG( LogSVODebugPathStepper, Verbose, TEXT( "OnProcessNeighbor From [%s] To [%s] with cost [%s]" ), *parent.NodeRef.ToString(), *neighbor.NodeRef.ToString(), *FString::SanitizeFloat( cost ) );
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnProcessNeighbor( const FGraphAStarDefaultNode< FSVOVolumeNavigationData > & neighbor )
{
    DebugInfos.ProcessedNeighbors.Emplace( FSVONodeAddressWithLocation( neighbor.ParentRef, Stepper.GetParameters().VolumeNavigationData ), FSVONodeAddressWithLocation( neighbor.NodeRef, Stepper.GetParameters().VolumeNavigationData ), neighbor.TotalCost, false );
    DebugInfos.VisitedNodes++;

    UE_LOG( LogSVODebugPathStepper, Verbose, TEXT( "OnProcessNeighbor From [%s] To [%s] with cost [%s]" ), *neighbor.ParentRef.ToString(), *neighbor.NodeRef.ToString(), *FString::SanitizeFloat( neighbor.TotalCost ) );
}

void FSVOPathFindingAStarObserver_GenerateDebugInfos::OnSearchSuccess( const ::TArray< FSVOPathFinderNodeAddressWithCost > & node_addresses )
{
    FillCurrentBestPath( node_addresses, true );

    const auto & params = Stepper.GetParameters();

    if ( params.QueryFilterSettings.bSmoothPaths )
    {
        SmoothPath( DebugInfos.CurrentBestPath, params.QueryFilterSettings.SmoothingSubdivisions );
    }

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

void FSVOPathFindingAStarObserver_GenerateDebugInfos::FillCurrentBestPath( const TArray< FSVOPathFinderNodeAddressWithCost > & node_addresses, const bool add_end_location ) const
{
    DebugInfos.CurrentBestPath.ResetForRepath();

    const auto & params = Stepper.GetParameters();

    BuildPath( DebugInfos.CurrentBestPath, params, node_addresses, add_end_location );

    DebugInfos.CurrentBestPath.MarkReady();
}
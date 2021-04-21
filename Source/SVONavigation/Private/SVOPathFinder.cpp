#include "SVOPathFinder.h"

#include "SVONavigationData.h"
#include "SVONavigationQueryFilterImpl.h"
#include "SVONavigationTypes.h"
#include "SVOPathCostCalculator.h"
#include "SVOPathHeuristicCalculator.h"

namespace
{
    void BuildPath( FNavigationPath & path, const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start_link, const FSVOOctreeLink & end_link, const TMap< FSVOOctreeLink, FSVOOctreeLink > & came_from )
    {
        auto & path_points = path.GetPathPoints();

        auto current = end_link;

        const auto get_link_position = []( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & link ) {
            FVector link_position;
            bounds_data.GetLinkPosition( link_position, link );
            return link_position;
        };

        while ( current != start_link )
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

        if ( path_points.Num() > 1 )
        {
            path_points.Top().Location = end_location;
        }
    }
}

FSVOPathFinder::FSVOPathFinder( const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FNavigationQueryFilter & navigation_query_filter ) :
    NavigationData( navigation_data ),
    StartLocation( start_location ),
    EndLocation( end_location ),
    NavigationQueryFilter( navigation_query_filter ),
    QueryFilterImplementation( static_cast< const FSVONavigationQueryFilterImpl * >( navigation_query_filter.GetImplementation() ) )
{
    const auto & query_filter_settings = QueryFilterImplementation->QueryFilterSettings;

    checkf( query_filter_settings.PathHeuristicCalculator != nullptr, TEXT( "Pathfinding the SVO requires the navigation query filter to have a path heuristic calculator defined" ) );
    checkf( query_filter_settings.PathCostCalculator != nullptr, TEXT( "Pathfinding the SVO requires the navigation query filter to have a path cost calculator defined" ) );

    HeuristicCalculator = Cast< USVOPathHeuristicCalculator >( query_filter_settings.PathHeuristicCalculator->ClassDefaultObject );
    CostCalculator = Cast< USVOPathCostCalculator >( query_filter_settings.PathCostCalculator->ClassDefaultObject );

    const auto & navigation_bounds_data = NavigationData.GetNavigationBoundsData();

    BoundsNavigationData = navigation_bounds_data.FindByPredicate( [ this, &start_location, &end_location ]( const FSVOBoundsNavigationData & data ) {
        return data.GetNavigationBounds().IsInside( start_location ) && data.GetNavigationBounds().IsInside( end_location );
    } );

    if ( BoundsNavigationData == nullptr )
    {
        return;
    }

    BoundsNavigationData->GetLinkFromPosition( StartLink, start_location );
    BoundsNavigationData->GetLinkFromPosition( EndLink, end_location );

    Frontier.Emplace( StartLink, HeuristicCalculator->GetHeuristicCost( *BoundsNavigationData, StartLink, EndLink ) * NavigationQueryFilter.GetHeuristicScale() );
    CameFrom.Add( StartLink, StartLink );
    CostSoFar.Add( StartLink, 0.0f );
}

ENavigationQueryResult::Type FSVOPathFinder::GetPath( FNavigationPath & navigation_path )
{
    if ( BoundsNavigationData == nullptr )
    {
        return ENavigationQueryResult::Fail;
    }

    int iterations = 0;

    while ( Frontier.Num() > 0 )
    {
        const auto current = Frontier.Pop();

        if ( current.Link == EndLink )
        {
            BuildPath( navigation_path, *BoundsNavigationData, StartLink, EndLink, CameFrom );
            AdjustPathEnds( navigation_path, StartLocation, EndLocation );
            navigation_path.MarkReady();

            return ENavigationQueryResult::Success;
        }

        TArray< FSVOOctreeLink > neighbors;
        BoundsNavigationData->GetNeighbors( neighbors, current.Link );

        for ( const auto & neighbor : neighbors )
        {
            const auto new_cost = CostSoFar[ current.Link ] + CostCalculator->GetCost( *BoundsNavigationData, current.Link, neighbor );
            if ( !CostSoFar.Contains( neighbor ) || new_cost < CostSoFar[ neighbor ] )
            {
                CostSoFar.FindOrAdd( neighbor ) = new_cost;
                const auto priority = new_cost + HeuristicCalculator->GetHeuristicCost( *BoundsNavigationData, neighbor, EndLink ) * NavigationQueryFilter.GetHeuristicScale();
                Frontier.Emplace( neighbor, priority );
                Frontier.Sort();
                CameFrom.FindOrAdd( neighbor ) = current.Link;
            }
        }

        iterations++;
    }

    return ENavigationQueryResult::Fail;
}

bool FSVOPathFinder::GetPathByStep( ENavigationQueryResult::Type & result, FNavigationPath & navigation_path, FSVOPathFinderDebugStep & step_debug )
{
    check( BoundsNavigationData != nullptr );

    if ( Frontier.Num() == 0 )
    {
        result = ENavigationQueryResult::Fail;
        return false;
    }

    const auto current = Frontier.Pop();

    step_debug.CurrentLocationCost.Cost = current.Cost;
    BoundsNavigationData->GetLinkPosition( step_debug.CurrentLocationCost.Location, current.Link );

    if ( current.Link == EndLink )
    {
        BuildPath( navigation_path, *BoundsNavigationData, StartLink, EndLink, CameFrom );
        AdjustPathEnds( navigation_path, StartLocation, EndLocation );
        navigation_path.MarkReady();

        result = ENavigationQueryResult::Success;
        return false;
    }

    TArray< FSVOOctreeLink > neighbors;
    BoundsNavigationData->GetNeighbors( neighbors, current.Link );

    for ( const auto & neighbor : neighbors )
    {
        const auto neighbor_cost = CostCalculator->GetCost( *BoundsNavigationData, current.Link, neighbor );
        const auto new_cost = CostSoFar[ current.Link ] + neighbor_cost;

        FSVOPathFinderDebugCost neighbor_debug_cost;
        neighbor_debug_cost.Cost = neighbor_cost;
        BoundsNavigationData->GetLinkPosition( neighbor_debug_cost.Location, neighbor );

        if ( !CostSoFar.Contains( neighbor ) || new_cost < CostSoFar[ neighbor ] )
        {
            CostSoFar.FindOrAdd( neighbor ) = new_cost;
            const auto priority = new_cost + HeuristicCalculator->GetHeuristicCost( *BoundsNavigationData, neighbor, EndLink ) * NavigationQueryFilter.GetHeuristicScale();
            Frontier.Emplace( neighbor, priority );
            Frontier.Sort();
            CameFrom.FindOrAdd( neighbor ) = current.Link;

            neighbor_debug_cost.WasEvaluated = true;
        }

        step_debug.NeighborLocationCosts.Emplace( MoveTemp( neighbor_debug_cost ) );
    }

    return true;
}
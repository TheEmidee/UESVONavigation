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

FSVOPathFinder::FSVOPathFinder( const ASVONavigationData & navigation_data ) :
    NavigationData( navigation_data )
{
}

ENavigationQueryResult::Type FSVOPathFinder::GetPath( FNavigationPath & navigation_path, const FVector & start_location, const FVector & end_location, const FNavigationQueryFilter & navigation_query_filter )
{
    const FSVONavigationQueryFilterImpl * query_filter_impl = static_cast< const FSVONavigationQueryFilterImpl * >( navigation_query_filter.GetImplementation() );

    if ( query_filter_impl == nullptr )
    {
        UE_LOG( LogNavigation, Error, TEXT( "Pathfinding the SVO requires the navigation query filter to have an implementation defined" ) );
        return ENavigationQueryResult::Error;
    }

    if ( query_filter_impl->PathHeuristicCalculator == nullptr )
    {
        UE_LOG( LogNavigation, Error, TEXT( "Pathfinding the SVO requires the navigation query filter to have a path heuristic calculator defined" ) );
        return ENavigationQueryResult::Error;
    }

    if ( query_filter_impl->PathCostCalculator == nullptr )
    {
        UE_LOG( LogNavigation, Error, TEXT( "Pathfinding the SVO requires the navigation query filter to have a path cost calculator defined" ) );
        return ENavigationQueryResult::Error;
    }

    const auto * heuristic_calculator = Cast< USVOPathHeuristicCalculator >( query_filter_impl->PathHeuristicCalculator->ClassDefaultObject );
    const auto * cost_calculator = Cast< USVOPathCostCalculator >( query_filter_impl->PathCostCalculator->ClassDefaultObject );

    const auto & navigation_bounds_data = NavigationData.GetNavigationBoundsData();

    const auto * bounds_data = navigation_bounds_data.FindByPredicate( [ this, &start_location, &end_location ]( const FSVOBoundsNavigationData & data ) {
        return data.GetNavigationBounds().IsInside( start_location ) && data.GetNavigationBounds().IsInside( end_location );
    } );

    if ( bounds_data == nullptr )
    {
        return ENavigationQueryResult::Fail;
    }

    FSVOOctreeLink start_link;
    bounds_data->GetLinkFromPosition( start_link, start_location );
    if ( !start_link.IsValid() )
    {
        return ENavigationQueryResult::Fail;
    }

    FSVOOctreeLink end_link;
    bounds_data->GetLinkFromPosition( end_link, end_location );
    if ( !end_link.IsValid() )
    {
        return ENavigationQueryResult::Fail;
    }

    struct FSVOLinkWithCost
    {
        FSVOLinkWithCost( const FSVOOctreeLink & link, float cost ) :
            Link( link ),
            Cost( cost )
        {
        }

        FSVOOctreeLink Link;
        float Cost;

        bool operator<( const FSVOLinkWithCost & other ) const
        {
            return Cost > other.Cost;
        }
    };

    TArray< FSVOLinkWithCost > frontier;
    TArray< FSVOOctreeLink > neighbors;
    TMap< FSVOOctreeLink, FSVOOctreeLink > came_from;
    TMap< FSVOOctreeLink, float > cost_so_far;

    frontier.Emplace( start_link, 0.0f );
    came_from.Add( start_link, start_link );
    cost_so_far.Add( start_link, 0.0f );

    while ( frontier.Num() > 0 )
    {
        const auto current = frontier.Pop();

        if ( current.Link == end_link )
        {
            BuildPath( navigation_path, *bounds_data, start_link, end_link, came_from );
            AdjustPathEnds( navigation_path, start_location, end_location );
            navigation_path.MarkReady();

            return ENavigationQueryResult::Success;
        }

        bounds_data->GetNeighbors( neighbors, current.Link );

        for ( const auto & next : neighbors )
        {
            const auto new_cost = cost_so_far[ current.Link ] + cost_calculator->GetCost( *bounds_data, current.Link, next );
            if ( !cost_so_far.Contains( next ) || new_cost < cost_so_far[ next ] )
            {
                cost_so_far.FindOrAdd( next ) = new_cost;
                const auto priority = new_cost + heuristic_calculator->GetHeuristicCost( *bounds_data, next, end_link );
                frontier.Emplace( next, priority );
                frontier.Sort();
                came_from.FindOrAdd( next ) = current.Link;
            }
        }
    }

    return ENavigationQueryResult::Fail;
}
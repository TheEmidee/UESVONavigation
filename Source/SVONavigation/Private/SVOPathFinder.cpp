#include "SVOPathFinder.h"

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

        for ( auto & path_point : path_points )
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

void FSVOPathFinderDebugInfos::Reset()
{
    DebugSteps.Reset();
    CurrentBestPath.ResetForRepath();
    Iterations = 0;
    VisitedNodes = 0;
}

FSVOPathFinder::FSVOPathFinder( const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query ) :
    NavigationData( navigation_data ),
    StartLocation( start_location ),
    EndLocation( end_location ),
    NavigationQueryFilter( *path_finding_query.QueryFilter ),
    QueryFilterImplementation( static_cast< const FSVONavigationQueryFilterImpl * >( path_finding_query.QueryFilter->GetImplementation() ) )
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

    BoundsNavigationData->GetLinkFromPosition( StartLink, StartLocation );
    BoundsNavigationData->GetLinkFromPosition( EndLink, EndLocation );

    if ( StartLink.IsValid() && EndLink.IsValid() )
    {
        Frontier.Emplace( StartLink, HeuristicCalculator->GetHeuristicCost( *BoundsNavigationData, StartLink, EndLink ) * NavigationQueryFilter.GetHeuristicScale() );
        CameFrom.Add( StartLink, StartLink );
        CostSoFar.Add( StartLink, 0.0f );
    }

    VerticalOffset = query_filter_settings.bOffsetPathVerticallyByAgentRadius
        ? -path_finding_query.NavAgentProperties.AgentRadius
        : 0.0f;
}

ENavigationQueryResult::Type FSVOPathFinder::GetPath( FNavigationPath & navigation_path )
{
    if ( BoundsNavigationData == nullptr 
        || !StartLink.IsValid() 
        || !EndLink.IsValid() 
        )
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
            if ( VerticalOffset != 0.0f )
            {
                ApplyVerticalOffset( navigation_path, VerticalOffset );
            }

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

bool FSVOPathFinder::GetPathByStep( ENavigationQueryResult::Type & result, FSVOPathFinderDebugInfos & debug_infos )
{
    if ( BoundsNavigationData == nullptr 
        || !StartLink.IsValid() 
        || !EndLink.IsValid()
        || Frontier.Num() == 0
        )
    {
        result = ENavigationQueryResult::Fail;
        return false;
    }

    const auto current = Frontier.Pop();

    FSVOPathFinderDebugStep debug_step;

    debug_step.CurrentLocationCost.Cost = current.Cost;
    BoundsNavigationData->GetLinkPosition( debug_step.CurrentLocationCost.Location, current.Link );

    if ( current.Link == EndLink )
    {
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

            debug_infos.CurrentBestPath.ResetForRepath();
            BuildPath( debug_infos.CurrentBestPath, *BoundsNavigationData, StartLink, current.Link, CameFrom );

            neighbor_debug_cost.WasEvaluated = true;
        }

        debug_step.NeighborLocationCosts.Emplace( MoveTemp( neighbor_debug_cost ) );
        debug_infos.VisitedNodes++;
    }

    debug_infos.DebugSteps.Emplace( MoveTemp( debug_step ) );
    debug_infos.Iterations++;

    return true;
}
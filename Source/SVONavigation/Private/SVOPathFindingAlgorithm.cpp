#include "SVOPathFindingAlgorithm.h"

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

        for (auto & path_point : path_points)
        {
            path_point.Location.Z += vertical_offset;
        }
    }

    void BuildPath( FNavigationPath & path, const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start_link, const FSVOOctreeLink & end_link, const TMap< FSVOOctreeLink, FSVOOctreeLink > & came_from )
    {
        auto & path_points = path.GetPathPoints();

        auto current = end_link;

        constexpr auto get_link_position = []( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & link ) {
            FVector link_position;
            bounds_data.GetLinkPosition( link_position, link );
            return link_position;
        };

        while (current != start_link)
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

        if (path_points.Num() > 1)
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

bool FSVOPathFindingAlgorithmStepper::Step( ENavigationQueryResult::Type & result, FSVOPathFinderDebugInfos & debug_infos )
{
    result = ENavigationQueryResult::Error;
    return false;
}

ENavigationQueryResult::Type USVOPathFindingAlgorithm::GetPath( FNavigationPath & navigation_path, const ASVONavigationData & /*navigation_data*/, const FVector & /*start_location*/, const FVector & /*end_location*/, const FPathFindingQuery & path_finding_query ) const
{
    navigation_path.ResetForRepath();
    return ENavigationQueryResult::Fail;
}

TSharedPtr<FSVOPathFindingAlgorithmStepper> USVOPathFindingAlgorithm::GetPathStepper() const
{
    return MakeShared< FSVOPathFindingAlgorithmStepper >();
}

bool FSVOPathFindingAlgorithmStepperAStar::Step( ENavigationQueryResult::Type & result, FSVOPathFinderDebugInfos & debug_infos )
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

ENavigationQueryResult::Type USVOPathFindingAlgorithmAStar::GetPath( FNavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query ) const
{
    const auto & navigation_query_filter = *path_finding_query.QueryFilter;
    const auto * query_filter_implementation = static_cast< const FSVONavigationQueryFilterImpl * >( path_finding_query.QueryFilter->GetImplementation() );
    const auto & query_filter_settings = query_filter_implementation->QueryFilterSettings;

    if (query_filter_settings.PathCostCalculatorClass == nullptr)
    {
        return ENavigationQueryResult::Error;
    }

    if (query_filter_settings.PathHeuristicCalculatorClass == nullptr)
    {
        return ENavigationQueryResult::Error;
    }

    const auto * heuristic_calculator = query_filter_settings.PathHeuristicCalculatorClass->GetDefaultObject< USVOPathHeuristicCalculator >();
    if (heuristic_calculator == nullptr)
    {
        return ENavigationQueryResult::Error;
    }

    const auto * cost_calculator = query_filter_settings.PathCostCalculatorClass->GetDefaultObject< USVOPathCostCalculator >();
    if (cost_calculator == nullptr)
    {
        return ENavigationQueryResult::Error;
    }

    const auto & svo_data = navigation_data.GetSVOData();
    const auto * bounds_navigation_data = svo_data.GetBoundsNavigationDataContainingPoints( { start_location, end_location } );

    if (bounds_navigation_data == nullptr)
    {
        return ENavigationQueryResult::Error;
    }

    FSVOOctreeLink start_link;
    bounds_navigation_data->GetLinkFromPosition( start_link, start_location );

    if (!start_link.IsValid())
    {
        return ENavigationQueryResult::Error;
    }

    FSVOOctreeLink end_link;
    bounds_navigation_data->GetLinkFromPosition( end_link, end_location );

    if (!end_link.IsValid())
    {
        return ENavigationQueryResult::Error;
    }

    TArray< FSVOLinkWithCost > frontier;
    TMap< FSVOOctreeLink, FSVOOctreeLink > came_from;
    TMap< FSVOOctreeLink, float > cost_so_far;

    frontier.Emplace( start_link, heuristic_calculator->GetHeuristicCost( *bounds_navigation_data, start_link, end_link ) * navigation_query_filter.GetHeuristicScale() );
    came_from.Add( start_link, start_link );
    cost_so_far.Add( start_link, 0.0f );

    const auto vertical_offset = query_filter_settings.bOffsetPathVerticallyByAgentRadius
                                    ? -path_finding_query.NavAgentProperties.AgentRadius
                                    : 0.0f;

    int iterations = 0;

    while (frontier.Num() > 0)
    {
        const auto current = frontier.Pop();

        if (current.Link == end_link)
        {
            BuildPath( navigation_path, *bounds_navigation_data, start_link, end_link, came_from );
            AdjustPathEnds( navigation_path, start_location, end_location );
            if (vertical_offset != 0.0f)
            {
                ApplyVerticalOffset( navigation_path, vertical_offset );
            }

            navigation_path.MarkReady();

            return ENavigationQueryResult::Success;
        }

        TArray< FSVOOctreeLink > neighbors;
        bounds_navigation_data->GetNeighbors( neighbors, current.Link );

        for (const auto & neighbor : neighbors)
        {
            const auto new_cost = cost_so_far[ current.Link ] + cost_calculator->GetCost( *bounds_navigation_data, current.Link, neighbor );
            if (!cost_so_far.Contains( neighbor ) || new_cost < cost_so_far[ neighbor ])
            {
                cost_so_far.FindOrAdd( neighbor ) = new_cost;
                const auto priority = new_cost + heuristic_calculator->GetHeuristicCost( *bounds_navigation_data, neighbor, end_link ) * navigation_query_filter.GetHeuristicScale();
                frontier.Emplace( neighbor, priority );
                frontier.Sort();
                came_from.FindOrAdd( neighbor ) = current.Link;
            }
        }

        iterations++;
    }

    return ENavigationQueryResult::Fail;
}

TSharedPtr<FSVOPathFindingAlgorithmStepper> USVOPathFindingAlgorithmAStar::GetPathStepper() const
{
    return MakeShared< FSVOPathFindingAlgorithmStepperAStar >();
}

// bool FSVOPathFinder::GetPathByStep( ENavigationQueryResult::Type & result, FSVOPathFinderDebugInfos & debug_infos )
// {
//     
// }

#include "PathFinding/SVOPathFinder.h"

#include "SVONavigationData.h"
#include "PathFinding/SVOPathFindingAlgorithm.h"
#include "Pathfinding/SVONavigationQueryFilterImpl.h"

namespace
{
    USVOPathFindingAlgorithm * GetPathFindingAlgorithm( const FSharedConstNavQueryFilter & nav_query_filter )
    {
        if ( !ensureAlwaysMsgf( nav_query_filter.IsValid(), TEXT( "The query filter is not valid" ) ) )
        {
            return nullptr;
        }

        const FSVONavigationQueryFilterImpl * query_filter_implementation = static_cast< const FSVONavigationQueryFilterImpl * >( nav_query_filter->GetImplementation() );

        if ( !ensureAlwaysMsgf( query_filter_implementation != nullptr, TEXT( "The query filter implementation is not valid" ) ) )
        {
            return nullptr;
        }

        const auto & query_filter_settings = query_filter_implementation->QueryFilterSettings;

        if ( !ensureAlwaysMsgf( query_filter_settings.PathFinder != nullptr, TEXT( "The PathFinder is not valid" ) ) )
        {
            return nullptr;
        }

        if ( !ensureAlwaysMsgf( query_filter_settings.TraversalCostCalculator != nullptr, TEXT( "The TraversalCostCalculator is not valid" ) ) )
        {
            return nullptr;
        }

        if ( !ensureAlwaysMsgf( query_filter_settings.HeuristicCalculator != nullptr, TEXT( "The HeuristicCalculator is not valid" ) ) )
        {
            return nullptr;
        }

        return query_filter_settings.PathFinder;
    }
}

ENavigationQueryResult::Type FSVOPathFinder::GetPath( FNavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FSharedConstNavQueryFilter & nav_query_filter )
{
    if ( const auto * path_finder = GetPathFindingAlgorithm( nav_query_filter ) )
    {
        if ( const auto * volume_navigation_data = navigation_data.GetVolumeNavigationDataContainingPoints( { start_location, end_location } ) )
        {
            const FSVOPathFindingParameters params( *volume_navigation_data, start_location, end_location, *nav_query_filter );
            return path_finder->GetPath( navigation_path, params );
        }
    }

    return ENavigationQueryResult::Fail;
}

TSharedPtr< FSVOPathFindingAlgorithmStepper > FSVOPathFinder::GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FSharedConstNavQueryFilter & nav_query_filter )
{
    if ( const auto * path_finder = GetPathFindingAlgorithm( nav_query_filter ) )
    {
        if ( const auto * volume_navigation_data = navigation_data.GetVolumeNavigationDataContainingPoints( { start_location, end_location } ) )
        {
            const FSVOPathFindingParameters params( *volume_navigation_data, start_location, end_location, *nav_query_filter );
            return path_finder->GetDebugPathStepper( debug_infos, params );
        }
    }

    return nullptr;
}

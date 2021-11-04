#include "SVOPathFinder.h"

#include "SVONavigationQueryFilterImpl.h"
#include "SVOPathFindingAlgorithm.h"

namespace
{
    USVOPathFindingAlgorithm * GetPathFindingAlgorithm( const FNavAgentProperties & agent_properties, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query )
    {
        if ( !ensureAlwaysMsgf( path_finding_query.QueryFilter.IsValid(), TEXT( "The query filter is not valid" ) ) )
        {
            return nullptr;
        }

        const FSVONavigationQueryFilterImpl * query_filter_implementation = static_cast< const FSVONavigationQueryFilterImpl * >( path_finding_query.QueryFilter->GetImplementation() );

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

ENavigationQueryResult::Type FSVOPathFinder::GetPath( FNavigationPath & navigation_path, const FNavAgentProperties & agent_properties, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query )
{
    if ( auto * path_finder = GetPathFindingAlgorithm( agent_properties, navigation_data, start_location, end_location, path_finding_query ) )
    {
        const FSVOPathFindingParameters params( agent_properties, navigation_data, start_location, end_location, path_finding_query );
        return path_finder->GetPath( navigation_path, params );
    }

    return ENavigationQueryResult::Fail;
}

TSharedPtr< FSVOPathFindingAlgorithmStepper > FSVOPathFinder::GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FNavAgentProperties & agent_properties, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query )
{
    if ( auto * path_finder = GetPathFindingAlgorithm( agent_properties, navigation_data, start_location, end_location, path_finding_query ) )
    {
        const FSVOPathFindingParameters params( agent_properties, navigation_data, start_location, end_location, path_finding_query );
        return path_finder->GetDebugPathStepper( debug_infos, params );
    }

    return nullptr;
}

#include "SVOPathFinder.h"

#include "SVONavigationQueryFilterImpl.h"
#include "SVOPathFindingAlgorithm.h"

ENavigationQueryResult::Type FSVOPathFinder::GetPath( FNavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query )
{
    if (!path_finding_query.QueryFilter.IsValid())
    {
        return ENavigationQueryResult::Error;
    }

    const FSVONavigationQueryFilterImpl * query_filter_implementation = static_cast< const FSVONavigationQueryFilterImpl * >( path_finding_query.QueryFilter->GetImplementation() );

    if (query_filter_implementation == nullptr)
    {
        return ENavigationQueryResult::Error;
    }

    const auto & query_filter_settings = query_filter_implementation->QueryFilterSettings;

    if (query_filter_settings.PathFinderClass == nullptr)
    {
        return ENavigationQueryResult::Error;
    }

    const FSVOPathFindingParameters params( navigation_data, start_location, end_location, path_finding_query );
    return query_filter_settings.PathFinderClass->GetDefaultObject< USVOPathFindingAlgorithm >()->GetPath( navigation_path, params );
}

TSharedPtr< FSVOPathFindingAlgorithmStepper > FSVOPathFinder::GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query )
{
    if (!path_finding_query.QueryFilter.IsValid())
    {
        return nullptr;
    }

    const FSVONavigationQueryFilterImpl * query_filter_implementation = static_cast< const FSVONavigationQueryFilterImpl * >( path_finding_query.QueryFilter->GetImplementation() );

    if (query_filter_implementation == nullptr)
    {
        return nullptr;
    }

    const auto & query_filter_settings = query_filter_implementation->QueryFilterSettings;

    if (query_filter_settings.PathFinderClass == nullptr)
    {
        return nullptr;
    }

    const FSVOPathFindingParameters params( navigation_data, start_location, end_location, path_finding_query );
    return query_filter_settings.PathFinderClass->GetDefaultObject< USVOPathFindingAlgorithm >()->GetDebugPathStepper( debug_infos, params );    
}

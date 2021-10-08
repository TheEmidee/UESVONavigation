#include "SVOPathFinder.h"

#include "SVONavigationQueryFilterImpl.h"
#include "SVOPathFindingAlgorithm.h"

ENavigationQueryResult::Type FSVOPathFinder::GetPath( FNavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query )
{
    if (!path_finding_query.QueryFilter.IsValid())
    {
        return ENavigationQueryResult::Error;
    }

    const FNavigationQueryFilter & nav_query_filter = *path_finding_query.QueryFilter;

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

    return query_filter_settings.PathFinderClass->GetDefaultObject< USVOPathFindingAlgorithm >()->GetPath( navigation_path, navigation_data, start_location, end_location, path_finding_query );
}

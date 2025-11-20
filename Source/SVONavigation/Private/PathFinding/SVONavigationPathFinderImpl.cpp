#include "PathFinding/SVONavigationPathFinderImpl.h"
#include "SVONavigationData.h"
#include "PathFinding/SVOPathFinder.h"

FPathFindingResult FSVONavigationPathFinderImpl::FindPath( const FNavAgentProperties & /*agent_properties*/, const FPathFindingQuery & path_finding_query )
{
    const auto * self = Cast< ASVONavigationData >( path_finding_query.NavData.Get() );

    if ( self == nullptr )
    {
        return ENavigationQueryResult::Error;
    }

    FPathFindingResult result( ENavigationQueryResult::Error );

    FNavigationPath * navigation_path = path_finding_query.PathInstanceToFill.Get();
    FSVONavigationPath * svo_navigation_path = navigation_path != nullptr
                                                   ? navigation_path->CastPath< FSVONavigationPath >()
                                                   : nullptr;

    if ( svo_navigation_path != nullptr )
    {
        result.Path = path_finding_query.PathInstanceToFill;
        svo_navigation_path->ResetForRepath();
    }
    else
    {
        result.Path = self->CreatePathInstance< FSVONavigationPath >( path_finding_query );
        navigation_path = result.Path.Get();
        svo_navigation_path = navigation_path != nullptr
                                  ? navigation_path->CastPath< FSVONavigationPath >()
                                  : nullptr;
    }

    if ( navigation_path != nullptr )
    {
        if ( path_finding_query.QueryFilter.IsValid() )
        {
            const FVector adjusted_end_location = path_finding_query.EndLocation; // navigation_filter->GetAdjustedEndLocation( path_finding_query.EndLocation );
            if ( ( path_finding_query.StartLocation - adjusted_end_location ).IsNearlyZero() )
            {
                result.Path->GetPathPoints().Reset();
                result.Path->GetPathPoints().Add( FNavPathPoint( adjusted_end_location ) );
                result.Result = ENavigationQueryResult::Success;
            }
            else
            {
                result.Result = FSVOPathFinder::GetPath( *svo_navigation_path, *self, path_finding_query.StartLocation, adjusted_end_location, path_finding_query.QueryFilter );
            }
        }
    }

    return result;
}
#pragma once

#include "SVOBoundsNavigationData.h"
#include "SVONavigationTypes.h"

#include <NavigationData.h>

class ASVONavigationData;

class SVONAVIGATION_API FSVOPathFinder
{
public:
    FSVOPathFinder( const ASVONavigationData & navigation_data );

    ENavigationQueryResult::Type GetPath( FNavigationPath & navigation_path, const FVector & start_location, const FVector & end_location, const FNavigationQueryFilter & navigation_query_filter );

private:
    ENavigationQueryResult::Type BuildPath( FNavigationPath & path, const ::FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start_link, const FSVOOctreeLink & end_link, const TMap< FSVOOctreeLink, FSVOOctreeLink > & came_from ) const;
    float GetHeuristicScore( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const;
    float GetCost( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const;

    const ASVONavigationData & NavigationData;
};

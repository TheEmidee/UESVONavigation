#pragma once

#include "SVOBoundsNavigationData.h"
#include "SVONavigationTypes.h"

#include <NavigationData.h>

class ASVONavigationData;

class SVONAVIGATION_API FSVOPathFinder
{
public:
    FSVOPathFinder( const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FNavigationQueryFilter & navigation_query_filter );

    FPathFindingResult GetPath( FNavigationPath & navigation_path );

private:
    void BuildPath( FNavigationPath & path, const FSVOOctreeLink & link, const FSVOBoundsNavigationData & bounds_navigation_data ) const;
    void ProcessLink( const FSVOOctreeLink & current_link, const FSVOOctreeLink & neighbor_link );
    float GetHeuristicScore( const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const;
    float GetCost( const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const;

    TSet< FSVOOctreeLink > OpenSet;
    TSet< FSVOOctreeLink > ClosedSet;
    TMap< FSVOOctreeLink, FSVOOctreeLink > Parent;
    TMap< FSVOOctreeLink, float > GScore;
    TMap< FSVOOctreeLink, float > FScore;
    FSVOOctreeLink EndLink;
    const FSVOBoundsNavigationData * BoundsData;
    const ASVONavigationData & NavigationData;
    const FVector & StartLocation;
    const FVector & EndLocation;
    const FNavigationQueryFilter & NavigationQueryFilter;
};

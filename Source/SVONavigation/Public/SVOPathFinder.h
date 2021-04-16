#pragma once

#include "SVONavigationTypes.h"

#include <NavigationData.h>

class ASVONavigationData;

class SVONAVIGATION_API FSVOPathFinder
{
public:
    FSVOPathFinder( const ASVONavigationData & navigation_data );

    ENavigationQueryResult::Type GetPath( FNavigationPath & navigation_path, const FVector & start_location, const FVector & end_location, const FNavigationQueryFilter & navigation_query_filter );

private:
    const ASVONavigationData & NavigationData;
};

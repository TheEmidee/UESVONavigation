#pragma once

#include "SVONavigationTypes.h"

#include <NavigationData.h>

class FSVOBoundsNavigationData;
class USVOPathCostCalculator;
class USVOPathHeuristicCalculator;
class FSVONavigationQueryFilterImpl;
class ASVONavigationData;

struct FSVOPathFinderDebugCost
{
    FVector Location = { 0.0f, 0.0f, 0.0f };
    float Cost = -1.0f;
    bool WasEvaluated = false;
};

struct FSVOPathFinderDebugStep
{
    FSVOPathFinderDebugCost CurrentLocationCost;
    TArray< FSVOPathFinderDebugCost, TInlineAllocator< 6 > > NeighborLocationCosts;
};

class SVONAVIGATION_API FSVOPathFinder
{
public:
    struct FSVOLinkWithCost
    {
        FSVOLinkWithCost( const FSVOOctreeLink & link, float cost ) :
            Link( link ),
            Cost( cost )
        {
        }

        FSVOOctreeLink Link;
        float Cost;

        bool operator<( const FSVOLinkWithCost & other ) const
        {
            return Cost > other.Cost;
        }
    };

    FSVOPathFinder( const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FNavigationQueryFilter & navigation_query_filter );

    ENavigationQueryResult::Type GetPath( FNavigationPath & navigation_path );
    bool GetPathByStep( ENavigationQueryResult::Type & result, FNavigationPath & navigation_path, FSVOPathFinderDebugStep & step_debug );

private:
    const ASVONavigationData & NavigationData;
    FVector StartLocation;
    FVector EndLocation;
    const FNavigationQueryFilter & NavigationQueryFilter;
    const FSVONavigationQueryFilterImpl * QueryFilterImplementation;
    const USVOPathHeuristicCalculator * HeuristicCalculator;
    const USVOPathCostCalculator * CostCalculator;
    const FSVOBoundsNavigationData * BoundsNavigationData;
    FSVOOctreeLink StartLink;
    FSVOOctreeLink EndLink;
    TArray< FSVOLinkWithCost > Frontier;
    TMap< FSVOOctreeLink, FSVOOctreeLink > CameFrom;
    TMap< FSVOOctreeLink, float > CostSoFar;
    FNavigationPath NavigationPath;
};

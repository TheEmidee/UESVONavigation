#pragma once

#include "SVONavigationTypes.h"

#include <NavigationData.h>

#include "SVOPathFinder.generated.h"

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

USTRUCT()
struct SVONAVIGATION_API FSVOPathFinderDebugInfos
{
    GENERATED_USTRUCT_BODY()
    
    void Reset();

    TArray< FSVOPathFinderDebugStep > DebugSteps;
    FNavigationPath CurrentBestPath;

    UPROPERTY( VisibleAnywhere )
    int Iterations;

    UPROPERTY( VisibleAnywhere )
    int VisitedNodes;
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

    FSVOPathFinder( const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query );

    ENavigationQueryResult::Type GetPath( FNavigationPath & navigation_path );
    bool GetPathByStep( ENavigationQueryResult::Type & result, FSVOPathFinderDebugInfos & debug_infos );

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
    float VerticalOffset;
};

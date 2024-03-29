#pragma once

#include "SVONavigationQueryFilterSettings.generated.h"

class USVOPathHeuristicCalculator;
class USVOPathTraversalCostCalculator;
class USVOPathFindingAlgorithm;

USTRUCT()
struct SVONAVIGATION_API FSVONavigationQueryFilterSettings
{
    GENERATED_USTRUCT_BODY()

    FSVONavigationQueryFilterSettings();

    UPROPERTY( EditAnywhere, Instanced )
    USVOPathFindingAlgorithm * PathFinder;

    UPROPERTY( EditAnywhere, Instanced )
    USVOPathTraversalCostCalculator * TraversalCostCalculator;

    UPROPERTY( EditAnywhere, Instanced )
    USVOPathHeuristicCalculator * HeuristicCalculator;

    UPROPERTY( EditDefaultsOnly )
    float HeuristicScale;

    // If set to true, this will lower the cost of traversing bigger nodes, and make the pathfinding more favorable traversing them
    UPROPERTY( EditDefaultsOnly )
    uint8 bUseNodeSizeCompensation : 1;

    UPROPERTY( EditDefaultsOnly )
    uint8 bSmoothPaths: 1;

    // How many intermediate points we will generate between the points returned by the pathfinding in order to smooth the curve (the bigger, the smoother)
    UPROPERTY( EditDefaultsOnly, meta = ( EditCondition = "bSmoothPaths == true" ) )
    int SmoothingSubdivisions;
};

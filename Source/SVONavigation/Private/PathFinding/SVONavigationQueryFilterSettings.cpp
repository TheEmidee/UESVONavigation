#include "Pathfinding/SVONavigationQueryFilterSettings.h"

#include "PathFinding/SVOPathFindingAlgorithm_LazyThetaStar.h"
#include "PathFinding/SVOPathHeuristicCalculator.h"
#include "PathFinding/SVOPathTraversalCostCalculator.h"

FSVONavigationQueryFilterSettings::FSVONavigationQueryFilterSettings() :
    PathFinder( NewObject< USVOPathFindingAlgorithmLazyThetaStar >() ),
    TraversalCostCalculator( NewObject< USVOPathCostCalculator_Fixed >() ),
    HeuristicCalculator( NewObject< USVOPathHeuristicCalculator_Manhattan >() ),
    HeuristicScale( 1.0f ),
    bUseNodeSizeCompensation( true ),
    bOffsetPathVerticallyByAgentRadius( true )
{
}

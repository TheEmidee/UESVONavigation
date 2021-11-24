#include "PathFinding/SVOPathTraversalCostCalculator.h"

#include "SVOVolumeNavigationData.h"

float USVOPathCostCalculator_Distance::GetTraversalCost( const FSVOVolumeNavigationData & bounds_data, const FSVONodeAddress & start, const FSVONodeAddress & end ) const
{
    const auto start_location = bounds_data.GetNodePositionFromAddress( start );
    const auto end_location = bounds_data.GetNodePositionFromAddress( end );
    const auto cost = ( start_location - end_location ).Size();

    return cost;
}

USVOPathCostCalculator_Fixed::USVOPathCostCalculator_Fixed() :
    Cost( 1.0f )
{
}

float USVOPathCostCalculator_Fixed::GetTraversalCost( const FSVOVolumeNavigationData & /*bounds_data*/, const FSVONodeAddress & /*start*/, const FSVONodeAddress & /*end */ ) const
{
    return Cost;
}

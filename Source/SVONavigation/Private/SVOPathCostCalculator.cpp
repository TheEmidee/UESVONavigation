#include "SVOPathCostCalculator.h"

#include "SVOBoundsNavigationData.h"

float USVOPathCostCalculator_Distance::GetCost( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const
{
    const auto start_location = bounds_data.GetLinkPosition( start );
    const auto end_location = bounds_data.GetLinkPosition( end );
    const auto cost = ( start_location - end_location ).Size();

    return cost;
}

float USVOPathCostCalculator_Fixed::GetCost( const FSVOBoundsNavigationData & /*bounds_data*/, const FSVOOctreeLink & /*start*/, const FSVOOctreeLink & /*end */ ) const
{
    return Cost;
}
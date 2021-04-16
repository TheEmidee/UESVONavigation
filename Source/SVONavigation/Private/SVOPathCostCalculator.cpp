#include "SVOPathCostCalculator.h"

#include "SVOBoundsNavigationData.h"

float USVOPathCostCalculator_Distance::GetCost( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const
{
    FVector start_location( 0.0f );
    FVector end_location( 0.0f );

    bounds_data.GetLinkPosition( start_location, start );
    bounds_data.GetLinkPosition( end_location, end );
    const auto cost = ( start_location - end_location ).Size();

    return cost;
}

float USVOPathCostCalculator_Fixed::GetCost( const FSVOBoundsNavigationData & /*bounds_data*/, const FSVOOctreeLink & /*start*/, const FSVOOctreeLink & /*end */ ) const
{
    return Cost;
}
#include "SVOHeuristicCalculator.h"

#include "SVOBoundsNavigationData.h"

float USVOPathHeuristicCalculator_Manhattan::GetHeuristicCost( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const
{
    const auto start_location = bounds_data.GetLinkPosition( start );
    const auto end_location = bounds_data.GetLinkPosition( end );
    const auto score = FMath::Abs( end_location.X - start_location.X ) + FMath::Abs( end_location.Y - start_location.Y ) + FMath::Abs( end_location.Z - start_location.Z );
    return score;
}

float USVOPathHeuristicCalculator_Euclidean::GetHeuristicCost( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const
{
    const auto start_location = bounds_data.GetLinkPosition( start );
    const auto end_location = bounds_data.GetLinkPosition( end );
    const auto score = ( start_location - end_location ).Size();

    return score;
}

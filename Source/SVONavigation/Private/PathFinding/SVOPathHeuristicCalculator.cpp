#include "PathFinding/SVOPathHeuristicCalculator.h"

#include "SVOVolumeNavigationData.h"

float USVOPathHeuristicCalculator_Manhattan::GetHeuristicCost( const FSVOVolumeNavigationData & bounds_data, const FSVONodeAddress & start, const FSVONodeAddress & end ) const
{
    const auto start_location = bounds_data.GetNodePositionFromAddress( start, true );
    const auto end_location = bounds_data.GetNodePositionFromAddress( end, true );
    const auto score = FMath::Abs( end_location.X - start_location.X ) + FMath::Abs( end_location.Y - start_location.Y ) + FMath::Abs( end_location.Z - start_location.Z );
    return score;
}

float USVOPathHeuristicCalculator_Euclidean::GetHeuristicCost( const FSVOVolumeNavigationData & bounds_data, const FSVONodeAddress & start, const FSVONodeAddress & end ) const
{
    const auto start_location = bounds_data.GetNodePositionFromAddress( start, true );
    const auto end_location = bounds_data.GetNodePositionFromAddress( end, true );
    const auto score = ( start_location - end_location ).Size();

    return score;
}

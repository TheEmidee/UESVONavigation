#include "PathFinding/SVONavigationPath.h"

FVector::FReal FSVONavigationPath::GetCostFromNode( NavNodeRef path_node ) const
{
    const auto index = PathPoints.IndexOfByPredicate( [ &path_node ]( const FNavPathPoint & nav_path_point ) {
        return nav_path_point.NodeRef == path_node;
    } );

    return GetCostFromIndex( index );
}

FVector::FReal FSVONavigationPath::GetCostFromIndex( const int32 path_point_index ) const
{
    if ( path_point_index < 0 || path_point_index >= PathPointCosts.Num() )
    {
        return 0.0f;
    }

    auto result = 0.f;

    for ( auto index = path_point_index; index < PathPointCosts.Num(); ++index )
    {
        result += PathPointCosts[ index ];
    }

    return result;
}

#pragma once

#include <NavigationPath.h>

struct SVONAVIGATION_API FSVONavigationPath final : public FNavigationPath
{
    typedef FNavigationPath Super;

    FSVONavigationPath() = default;

    TArray< float > & GetPathPointCosts();
    float GetCostFromNode( NavNodeRef path_node ) const override;
    float GetCostFromIndex( int32 path_point_index ) const override;

private:

    TArray< float > PathPointCosts;
};

FORCEINLINE TArray< float > & FSVONavigationPath::GetPathPointCosts()
{
    return PathPointCosts;
}

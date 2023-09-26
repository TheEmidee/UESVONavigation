#pragma once

#include <NavigationPath.h>

struct SVONAVIGATION_API FSVONavigationPath final : public FNavigationPath
{
    typedef FNavigationPath Super;

    FSVONavigationPath() = default;

    TArray< float > & GetPathPointCosts();
    FVector::FReal GetCostFromNode( NavNodeRef path_node ) const override;
    FVector::FReal GetCostFromIndex( int32 path_point_index ) const override;

private:
    TArray< float > PathPointCosts;
};

FORCEINLINE TArray< float > & FSVONavigationPath::GetPathPointCosts()
{
    return PathPointCosts;
}

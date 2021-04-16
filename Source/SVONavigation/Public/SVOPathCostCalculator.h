#pragma once

#include "SVOPathCostCalculator.generated.h"

struct FSVOOctreeLink;
class FSVOBoundsNavigationData;

UCLASS( abstract, NotBlueprintable )
class SVONAVIGATION_API USVOPathCostCalculator : public UObject
{
    GENERATED_BODY()

public:
    virtual float GetCost( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const PURE_VIRTUAL( USVOPathCostCalculator::GetCost, return 0.0f; );
};

UCLASS()
class SVONAVIGATION_API USVOPathCostCalculator_Distance final : public USVOPathCostCalculator
{
    GENERATED_BODY()

public:

    float GetCost( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const override;
};

UCLASS( Blueprintable )
class SVONAVIGATION_API USVOPathCostCalculator_Fixed final : public USVOPathCostCalculator
{
    GENERATED_BODY()

public:

    float GetCost( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const override;

private:

    UPROPERTY( EditDefaultsOnly )
    float Cost;
};
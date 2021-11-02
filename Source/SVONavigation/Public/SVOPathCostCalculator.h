#pragma once

#include "SVOPathCostCalculator.generated.h"

struct FSVOOctreeLink;
class FSVOBoundsNavigationData;

UCLASS( abstract, NotBlueprintable, EditInlineNew )
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

/*
 * Applies a fixed cost to node traversal.
 * This  means  that  no  matter  how  big  the  node  is,  traveling  through it has the same cost.
 * This effectively biases the search even more toward exploring through large nodes.
 */
UCLASS()
class SVONAVIGATION_API USVOPathCostCalculator_Fixed final : public USVOPathCostCalculator
{
    GENERATED_BODY()

public:

    USVOPathCostCalculator_Fixed();

    float GetCost( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const override;

private:

    UPROPERTY( EditDefaultsOnly )
    float Cost;
};
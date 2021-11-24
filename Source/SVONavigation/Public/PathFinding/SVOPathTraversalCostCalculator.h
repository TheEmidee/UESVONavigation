#pragma once

#include "SVOPathTraversalCostCalculator.generated.h"

struct FSVONodeAddress;
class FSVOVolumeNavigationData;

UCLASS( abstract, NotBlueprintable, EditInlineNew )
class SVONAVIGATION_API USVOPathTraversalCostCalculator : public UObject
{
    GENERATED_BODY()

public:
    virtual float GetTraversalCost( const FSVOVolumeNavigationData & bounds_data, const FSVONodeAddress & start, const FSVONodeAddress & end ) const PURE_VIRTUAL( USVOPathCostCalculator::GetCost, return 0.0f; );
};

UCLASS()
class SVONAVIGATION_API USVOPathCostCalculator_Distance final : public USVOPathTraversalCostCalculator
{
    GENERATED_BODY()

public:

    float GetTraversalCost( const FSVOVolumeNavigationData & bounds_data, const FSVONodeAddress & start, const FSVONodeAddress & end ) const override;
};

/*
 * Applies a fixed cost to node traversal.
 * This  means  that  no  matter  how  big  the  node  is,  traveling  through it has the same cost.
 * This effectively biases the search even more toward exploring through large nodes.
 */
UCLASS()
class SVONAVIGATION_API USVOPathCostCalculator_Fixed final : public USVOPathTraversalCostCalculator
{
    GENERATED_BODY()

public:

    USVOPathCostCalculator_Fixed();

    float GetTraversalCost( const FSVOVolumeNavigationData & bounds_data, const FSVONodeAddress & start, const FSVONodeAddress & end ) const override;

private:

    UPROPERTY( EditDefaultsOnly )
    float Cost;
};
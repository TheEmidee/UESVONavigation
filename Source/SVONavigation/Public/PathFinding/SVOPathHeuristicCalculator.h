#pragma once

#include "SVOPathHeuristicCalculator.generated.h"

class FSVOVolumeNavigationData;
struct FSVONodeAddress;

UCLASS( abstract, NotBlueprintable, EditInlineNew )
class SVONAVIGATION_API USVOPathHeuristicCalculator : public UObject
{
    GENERATED_BODY()

public:

    virtual float GetHeuristicCost( const FSVOVolumeNavigationData & bounds_data, const FSVONodeAddress & start, const FSVONodeAddress & end ) const PURE_VIRTUAL( USVOPathHeuristicCalculator::GetHeuristicCost, return 0.0f; );
};

UCLASS()
class SVONAVIGATION_API USVOPathHeuristicCalculator_Manhattan final : public USVOPathHeuristicCalculator
{
    GENERATED_BODY()

public:

    float GetHeuristicCost( const FSVOVolumeNavigationData & bounds_data, const FSVONodeAddress & start, const FSVONodeAddress & end ) const override;
};

UCLASS()
class SVONAVIGATION_API USVOPathHeuristicCalculator_Euclidean final : public USVOPathHeuristicCalculator
{
    GENERATED_BODY()

public:

    float GetHeuristicCost( const FSVOVolumeNavigationData & bounds_data, const FSVONodeAddress & start, const FSVONodeAddress & end ) const override;
};
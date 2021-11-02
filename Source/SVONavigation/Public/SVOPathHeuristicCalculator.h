#pragma once

#include "SVOPathHeuristicCalculator.generated.h"

class FSVOBoundsNavigationData;
struct FSVOOctreeLink;

UCLASS( abstract, NotBlueprintable, EditInlineNew )
class SVONAVIGATION_API USVOPathHeuristicCalculator : public UObject
{
    GENERATED_BODY()

public:

    virtual float GetHeuristicCost( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const PURE_VIRTUAL( USVOPathHeuristicCalculator::GetHeuristicCost, return 0.0f; );
};

UCLASS()
class SVONAVIGATION_API USVOPathHeuristicCalculator_Manhattan final : public USVOPathHeuristicCalculator
{
    GENERATED_BODY()

public:

    float GetHeuristicCost( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const override;
};

UCLASS()
class SVONAVIGATION_API USVOPathHeuristicCalculator_Euclidean final : public USVOPathHeuristicCalculator
{
    GENERATED_BODY()

public:

    float GetHeuristicCost( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const override;
};
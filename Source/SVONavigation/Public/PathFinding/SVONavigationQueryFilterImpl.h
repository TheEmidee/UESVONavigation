#pragma once

#include "SVONavigationQueryFilterSettings.h"

#include <AI/Navigation/NavQueryFilter.h>

class SVONAVIGATION_API FSVONavigationQueryFilterImpl final : public INavigationQueryFilterInterface
{
public:
    void Reset() override;
    void SetAreaCost( uint8 AreaType, float Cost ) override;
    void SetFixedAreaEnteringCost( uint8 AreaType, float Cost ) override;
    void SetExcludedArea( uint8 AreaType ) override;
    void SetAllAreaCosts( const float * CostArray, const int32 Count ) override;
    void GetAllAreaCosts( float * CostArray, float * FixedCostArray, const int32 Count ) const override;
    void SetBacktrackingEnabled( const bool bBacktracking ) override;
    bool IsBacktrackingEnabled() const override;
    float GetHeuristicScale() const override;
    bool IsEqual( const INavigationQueryFilterInterface * Other ) const override;
    void SetIncludeFlags( uint16 Flags ) override;
    uint16 GetIncludeFlags() const override;
    void SetExcludeFlags( uint16 Flags ) override;
    uint16 GetExcludeFlags() const override;
    INavigationQueryFilterInterface * CreateCopy() const override;

    FSVONavigationQueryFilterSettings QueryFilterSettings;
};
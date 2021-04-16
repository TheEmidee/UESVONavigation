#include "SVONavigationQueryFilterImpl.h"

void FSVONavigationQueryFilterImpl::Reset()
{
}

void FSVONavigationQueryFilterImpl::SetAreaCost( uint8 AreaType, float Cost )
{
}

void FSVONavigationQueryFilterImpl::SetFixedAreaEnteringCost( uint8 AreaType, float Cost )
{
}

void FSVONavigationQueryFilterImpl::SetExcludedArea( uint8 AreaType )
{
}

void FSVONavigationQueryFilterImpl::SetAllAreaCosts( const float * CostArray, const int32 Count )
{
}

void FSVONavigationQueryFilterImpl::GetAllAreaCosts( float * CostArray, float * FixedCostArray, const int32 Count ) const
{
}

void FSVONavigationQueryFilterImpl::SetBacktrackingEnabled( const bool bBacktracking )
{
}

bool FSVONavigationQueryFilterImpl::IsBacktrackingEnabled() const
{
    return false;
}

float FSVONavigationQueryFilterImpl::GetHeuristicScale() const
{
    return QueryFilterSettings.HeuristicScale;
}

bool FSVONavigationQueryFilterImpl::IsEqual( const INavigationQueryFilterInterface * Other ) const
{
    return Other == this;
}

void FSVONavigationQueryFilterImpl::SetIncludeFlags( uint16 Flags )
{
}

uint16 FSVONavigationQueryFilterImpl::GetIncludeFlags() const
{
    return 0;
}

void FSVONavigationQueryFilterImpl::SetExcludeFlags( uint16 Flags )
{
}

uint16 FSVONavigationQueryFilterImpl::GetExcludeFlags() const
{
    return 0;
}

INavigationQueryFilterInterface * FSVONavigationQueryFilterImpl::CreateCopy() const
{
    return new FSVONavigationQueryFilterImpl( *this );
}

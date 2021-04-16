#include "SVONavigationQueryFilter.h"

#include "SVONavigationQueryFilterImpl.h"

void USVONavigationQueryFilter::InitializeFilter( const ANavigationData & nav_data, const UObject * querier, FNavigationQueryFilter & filter ) const
{
    Super::InitializeFilter( nav_data, querier, filter );

    FSVONavigationQueryFilterImpl * filter_impl = static_cast< FSVONavigationQueryFilterImpl * >( filter.GetImplementation() );

    filter_impl->PathHeuristicCalculator = PathHeuristicCalculator;
    filter_impl->PathHeuristicCalculator = PathHeuristicCalculator;
}

#pragma once

#include "SVONavigationTypes.h"

#include <NavFilters/NavigationQueryFilter.h>

#include "SVONavigationQueryFilter.generated.h"

UCLASS()
class SVONAVIGATION_API USVONavigationQueryFilter final : public UNavigationQueryFilter
{
    GENERATED_BODY()

protected:

    void InitializeFilter( const ANavigationData & nav_data, const UObject * querier , FNavigationQueryFilter & filter ) const override;

private:

    UPROPERTY( EditDefaultsOnly )
    FSVONavigationQueryFilterSettings QueryFilterSettings;
};

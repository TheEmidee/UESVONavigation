#pragma once

#include <NavFilters/NavigationQueryFilter.h>

#include "SVONavigationQueryFilter.generated.h"

class USVOPathHeuristicCalculator;
class USVOPathCostCalculator;
UCLASS()
class SVONAVIGATION_API USVONavigationQueryFilter final : public UNavigationQueryFilter
{
    GENERATED_BODY()

public:

    USVONavigationQueryFilter();

protected:

    void InitializeFilter( const ANavigationData & nav_data, const UObject * querier , FNavigationQueryFilter & filter ) const override;

private:

    UPROPERTY( EditDefaultsOnly )
    TSubclassOf< USVOPathCostCalculator > PathCostCalculator;

    UPROPERTY( EditDefaultsOnly )
    TSubclassOf< USVOPathHeuristicCalculator > PathHeuristicCalculator;

    UPROPERTY( EditDefaultsOnly )
    float HeuristicScale;
};

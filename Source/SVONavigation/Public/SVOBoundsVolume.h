#pragma once

#include <CoreMinimal.h>
#include <NavMesh/NavMeshBoundsVolume.h>

#include "SVOBoundsVolume.generated.h"

class USVONavigationQueryFilter;

UCLASS()
class SVONAVIGATION_API ASVOBoundsVolume final : public ANavMeshBoundsVolume
{
    GENERATED_BODY()

public:
    const TSubclassOf< USVONavigationQueryFilter > & GetVolumeNavigationQueryFilter() const;

private:
    UPROPERTY( EditInstanceOnly )
    TSubclassOf< USVONavigationQueryFilter > VolumeNavigationQueryFilter;
};

FORCEINLINE const TSubclassOf< USVONavigationQueryFilter > & ASVOBoundsVolume::GetVolumeNavigationQueryFilter() const
{
    return VolumeNavigationQueryFilter;
}
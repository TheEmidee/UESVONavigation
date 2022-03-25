#pragma once

#include "SVOVolumeNavigationData.h"

#include <AI/Navigation/NavigationDataChunk.h>
#include <CoreMinimal.h>

#include "SVONavigationDataChunk.generated.h"

UCLASS()
class SVONAVIGATION_API USVONavigationDataChunk final : public UNavigationDataChunk
{
    GENERATED_BODY()

public:
    void Serialize( FArchive & archive ) override;
    void AddNavigationData( FSVOVolumeNavigationData & navigation_data );
    void ReleaseNavigationData();

    TArray< FSVOVolumeNavigationData > NavigationData;
};

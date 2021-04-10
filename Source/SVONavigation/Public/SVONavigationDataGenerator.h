#pragma once

#include <AI/NavDataGenerator.h>
#include <CoreMinimal.h>

#include "SVONavigationTypes.h"

class ASVONavigationData;

class SVONAVIGATION_API FSVONavigationDataGenerator : public FNavDataGenerator, public FNoncopyable
{
public:
    explicit FSVONavigationDataGenerator( ASVONavigationData & navigation_data );
    virtual ~FSVONavigationDataGenerator();

    void Init();

private:

    const ASVONavigationData * GetOwner() const;
    UWorld * GetWorld() const;
    bool MarkNavBoundsDirty();
    void UpdateNavigationBounds();

    ASVONavigationData & NavigationData;
    FSVODataBuildConfig BuildConfig;
    int MaxBoxGeneratorTasks;
    uint8 IsInitialized : 1;

    /** Total bounding box that includes all volumes, in unreal units. */
    FBox TotalNavBounds;

    /** Bounding geometry definition. */
    TNavStatArray< FBox > InclusionBounds;
};

FORCEINLINE const ASVONavigationData * FSVONavigationDataGenerator::GetOwner() const
{
    return &NavigationData;
}

FORCEINLINE UWorld * FSVONavigationDataGenerator::GetWorld() const
{
    return NavigationData.GetWorld();
}
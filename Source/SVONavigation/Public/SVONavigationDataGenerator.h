#pragma once

#include "SVONavigationData.h"
#include "SVONavigationTypes.h"
#include "Chaos/AABB.h"
#include "Chaos/AABB.h"


#include <AI/NavDataGenerator.h>

class ASVONavigationData;

class FSVONavigationDataGenerator;

struct FSVOBoundsNavigationDataGenerator final : public FNoncopyable
{
public:
    FSVOBoundsNavigationDataGenerator( FSVONavigationDataGenerator & navigation_data_generator, const FBox & volume_bounds );

    FSVOBoundsNavigationData GetBoundsNavigationData() const;

    bool DoWork();

private:
    FSVONavigationDataGenerator & ParentGenerator;
    FSVOBoundsNavigationData BoundsNavigationData;
    FBox VolumeBounds;
    TWeakObjectPtr< UWorld > World;
    FNavDataConfig NavDataConfig;
};

FORCEINLINE FSVOBoundsNavigationData FSVOBoundsNavigationDataGenerator::GetBoundsNavigationData() const
{
    return BoundsNavigationData;
}

struct SVONAVIGATION_API FSVOBoxGeneratorWrapper : public FNonAbandonableTask
{
    TSharedRef< FSVOBoundsNavigationDataGenerator > BoxNavigationDataGenerator;

    FSVOBoxGeneratorWrapper( TSharedRef< FSVOBoundsNavigationDataGenerator > box_navigation_generator ) :
        BoxNavigationDataGenerator( box_navigation_generator )
    {
    }

    void DoWork()
    {
        BoxNavigationDataGenerator->DoWork();
    }

    FORCEINLINE TStatId GetStatId() const
    {
        RETURN_QUICK_DECLARE_CYCLE_STAT( FSVOBoxGenerator, STATGROUP_ThreadPoolAsyncTasks );
    }
};

typedef FAsyncTask< FSVOBoxGeneratorWrapper > FSVOBoxGeneratorTask;

struct FPendingBoundsDataGenerationElement
{
    FBox VolumeBounds;
    float SeedDistance;

    FPendingBoundsDataGenerationElement() :
        VolumeBounds( EForceInit::ForceInit ),
        SeedDistance( MAX_flt )
    {
    }

    bool operator==( const FBox & other_box ) const
    {
        return VolumeBounds == other_box;
    }

    bool operator==( const FPendingBoundsDataGenerationElement & other ) const
    {
        return VolumeBounds == other.VolumeBounds;
    }

    bool operator<( const FPendingBoundsDataGenerationElement & other ) const
    {
        return other.SeedDistance < SeedDistance;
    }

    friend uint32 GetTypeHash( const FPendingBoundsDataGenerationElement & element )
    {
        return HashCombine( GetTypeHash( element.VolumeBounds.GetCenter() ), GetTypeHash( element.VolumeBounds.GetExtent() ) );
    }
};

struct FRunningBoundsDataGenerationElement
{
    FRunningBoundsDataGenerationElement() :
        VolumeBounds( EForceInit::ForceInit ),
        ShouldDiscard( false ),
        AsyncTask( nullptr )
    {
    }

    FRunningBoundsDataGenerationElement( const FBox & volume_bounds ) :
        VolumeBounds( volume_bounds ),
        ShouldDiscard( false ),
        AsyncTask( nullptr )
    {
    }

    bool operator==( const FRunningBoundsDataGenerationElement & other ) const
    {
        return VolumeBounds == other.VolumeBounds;
    }

    FBox VolumeBounds;
    /** whether generated results should be discarded */
    bool ShouldDiscard;
    FSVOBoxGeneratorTask * AsyncTask;
};

class SVONAVIGATION_API FSVONavigationDataGenerator final : public FNavDataGenerator, public FNoncopyable
{
public:
    explicit FSVONavigationDataGenerator( ASVONavigationData & navigation_data );

    const ASVONavigationData * GetOwner() const;
    UWorld * GetWorld() const;
    const FSVODataGenerationSettings & GetGenerationSettings() const;

    void Init();

    bool RebuildAll() override;
    void EnsureBuildCompletion() override;
    void CancelBuild() override;
    void TickAsyncBuild( float delta_seconds ) override;
    void OnNavigationBoundsChanged() override;
    void RebuildDirtyAreas( const TArray< FNavigationDirtyArea > & dirty_areas ) override;
    bool IsBuildInProgressCheckDirty() const override;
    int32 GetNumRemaningBuildTasks() const override;
    int32 GetNumRunningBuildTasks() const override;

private:
    void GetSeedLocations( TArray< FVector2D > & seed_locations, UWorld & world ) const;
    void SortPendingBounds();
    void UpdateNavigationBounds();
    TArray< FBox > ProcessAsyncTasks( int32 task_to_process_count );
    TSharedRef< FSVOBoundsNavigationDataGenerator > CreateBoxNavigationGenerator( const FBox & box );

    ASVONavigationData & NavigationData;
    FSVODataGenerationSettings GenerationSettings;
    int MaximumGeneratorTaskCount;
    uint8 IsInitialized : 1;

    /** Total bounding box that includes all volumes, in unreal units. */
    FBox TotalNavigationBounds;

    TNavStatArray< FBox > RegisteredNavigationBounds;
    TNavStatArray< FPendingBoundsDataGenerationElement > PendingBoundsDataGenerationElements;
    TNavStatArray< FRunningBoundsDataGenerationElement > RunningBoundsDataGenerationElements;
};

FORCEINLINE const ASVONavigationData * FSVONavigationDataGenerator::GetOwner() const
{
    return &NavigationData;
}

FORCEINLINE UWorld * FSVONavigationDataGenerator::GetWorld() const
{
    return NavigationData.GetWorld();
}

FORCEINLINE const FSVODataGenerationSettings & FSVONavigationDataGenerator::GetGenerationSettings() const
{
    return GenerationSettings;
}
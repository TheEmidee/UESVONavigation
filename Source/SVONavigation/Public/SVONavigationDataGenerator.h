#pragma once

#include "SVONavigationTypes.h"

#include <AI/NavDataGenerator.h>

class ASVONavigationData;

class FSVONavigationDataGenerator;

struct FSVOBoxNavigationDataGenerator : public FNoncopyable
{
public:
    FSVOBoxNavigationDataGenerator( FSVONavigationDataGenerator & navigation_data_generator, const FBox & box );

    bool DoWork();

private:
    FBox Box;
    FSVONavigationBoundsData NavigationBoundsData;
    TWeakPtr< FNavDataGenerator, ESPMode::ThreadSafe > ParentGeneratorWeakPtr;
};

struct SVONAVIGATION_API FSVOBoxGeneratorWrapper : public FNonAbandonableTask
{
    TSharedRef< FSVOBoxNavigationDataGenerator > BoxNavigationDataGenerator;

    FSVOBoxGeneratorWrapper( TSharedRef< FSVOBoxNavigationDataGenerator > box_navigation_generator ) :
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

struct FPendingBoxElement
{
    FBox Box;
    float SeedDistance;

    FPendingBoxElement() :
        Box( EForceInit::ForceInit ),
        SeedDistance( MAX_flt )
    {
    }

    bool operator==( const FBox & other_box ) const
    {
        return Box == other_box;
    }

    bool operator==( const FPendingBoxElement & other ) const
    {
        return Box == other.Box;
    }

    bool operator<( const FPendingBoxElement & other ) const
    {
        return other.SeedDistance < SeedDistance;
    }

    friend uint32 GetTypeHash( const FPendingBoxElement & element )
    {
        return HashCombine( GetTypeHash( element.Box.GetCenter() ), GetTypeHash( element.Box.GetExtent() ) );
    }
};

struct FRunningBoxElement
{
    FRunningBoxElement() :
        Box( EForceInit::ForceInit ),
        ShouldDiscard( false ),
        AsyncTask( nullptr )
    {
    }

    FRunningBoxElement( const FBox & box ) :
        Box( box ),
        ShouldDiscard( false ),
        AsyncTask( nullptr )
    {
    }

    bool operator==( const FRunningBoxElement & other ) const
    {
        return Box == other.Box;
    }

    FBox Box;
    /** whether generated results should be discarded */
    bool ShouldDiscard;
    FSVOBoxGeneratorTask * AsyncTask;
};

class SVONAVIGATION_API FSVONavigationDataGenerator : public FNavDataGenerator, public FNoncopyable
{
public:
    explicit FSVONavigationDataGenerator( ASVONavigationData & navigation_data );
    virtual ~FSVONavigationDataGenerator();

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
    uint32 LogMemUsed() const override;

private:
    const ASVONavigationData * GetOwner() const;
    UWorld * GetWorld() const;
    void UpdateNavigationBounds();
    TArray< FBox > ProcessAsyncTasks( int32 task_to_process_count );
    TSharedRef< FSVOBoxNavigationDataGenerator > CreateBoxNavigationGenerator( const FBox & box );

    ASVONavigationData & NavigationData;
    FSVODataBuildConfig BuildConfig;
    int MaxBoxGeneratorTasks;
    uint8 IsInitialized : 1;

    /** Total bounding box that includes all volumes, in unreal units. */
    FBox TotalNavBounds;

    /** Bounding geometry definition. */
    TNavStatArray< FBox > RegisteredBounds;

    TNavStatArray< FPendingBoxElement > PendingDirtyBoxes;

    /** List of dirty tiles currently being regenerated */
    TNavStatArray< FRunningBoxElement > RunningDirtyBoxes;
};

FORCEINLINE const ASVONavigationData * FSVONavigationDataGenerator::GetOwner() const
{
    return &NavigationData;
}

FORCEINLINE UWorld * FSVONavigationDataGenerator::GetWorld() const
{
    return NavigationData.GetWorld();
}
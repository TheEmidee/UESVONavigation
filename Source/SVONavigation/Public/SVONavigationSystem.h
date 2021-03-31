#pragma once

#include "SVONavigationVolume.h"

#include <Subsystems/EngineSubsystem.h>

#include "SVONavigationSystem.generated.h"

class ASVONavigationData;

struct FSVONavigationBounds
{
    uint32 UniqueID;
    TWeakObjectPtr< ULevel > Level; // The level this bounds belongs to
    FBox AreaBox;

    bool operator==( const FSVONavigationBounds & Other ) const
    {
        return UniqueID == Other.UniqueID;
    }

    friend uint32 GetTypeHash( const FSVONavigationBounds & NavBounds )
    {
        return GetTypeHash( NavBounds.UniqueID );
    }
};

struct FSVONavigationBoundsUpdateRequest
{
    FSVONavigationBounds NavBounds;

    enum class Type : uint8
    {
        Added,
        Removed,
        Updated,
    };

    Type UpdateRequest;
};

UCLASS()
class SVONAVIGATION_API USVONavigationSystem : public UEngineSubsystem
{
    GENERATED_BODY()

public:

    void Initialize( FSubsystemCollectionBase & collection ) override;
    void Deinitialize() override;
    UWorld * GetWorld() const override;
    void OnNavigationVolumeAdded( const ASVONavigationVolume & volume );
    void OnNavigationVolumeRemoved( const ASVONavigationVolume & volume );
    void OnNavigationVolumeUpdated( const ASVONavigationVolume & volume );
    bool Tick( float delta_seconds );

private:

#if WITH_EDITOR
    void OnActorMoved( AActor * actor );
#endif

    void RegisterWorldDelegates();
    void UnRegisterWorldDelegates();
    void OnPostWorldInitialized( UWorld * world, const UWorld::InitializationValues initialization_values );
    void OnPostWorldCleanup( UWorld * world, bool session_ended, bool cleanup_resources );
    void OnLevelAddedToWorld( ULevel * level, UWorld * world );
    void OnLevelRemovedFromWorld( ULevel * level, UWorld * world );
    void OnPostLoadMap( UWorld * world );
    ASVONavigationData * CreateNavigationData() const;
    void GatherExistingNavigationData();
    void SpawnMissingNavigationData();
    ASVONavigationData * GetDefaultNavigationDataInstance( bool it_creates_if_missing );
    void PerformNavigationVolumesUpdate( const TArray< FSVONavigationBoundsUpdateRequest > & update_requests );
    void AddNavigationVolumeUpdateRequest( const FSVONavigationBoundsUpdateRequest & update_request );
    bool IsThereAnywhereToBuildNavigation() const;

    UPROPERTY( Transient )
    TWeakObjectPtr< ASVONavigationData > NavigationData;

    UPROPERTY( Transient )
    TWeakObjectPtr< UWorld > World;

    TArray< FSVONavigationBoundsUpdateRequest > PendingNavigationVolumeUpdateRequests;
    /** All areas where we build/have navigation */
    TSet< FSVONavigationBounds > RegisteredNavigationBounds;
    FTickerDelegate TickDelegate;
    FDelegateHandle TickDelegateHandle;
    FDelegateHandle PostLoadMapWithWorldDelegateHandle;
    FDelegateHandle OnPostWorldInitializationDelegateHandle;
    FDelegateHandle OnPostWorldCleanupDelegateHandle;
    FDelegateHandle OnLevelAddedToWorldDelegateHandle;
    FDelegateHandle OnLevelRemovedFromWorldDelegateHandle;
};

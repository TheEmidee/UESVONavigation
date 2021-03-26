#pragma once

#include "SVONavigationVolume.h"

#include <Subsystems/EngineSubsystem.h>

#include "SVONavigationSystem.generated.h"

struct FSVONavigationBounds
{
    uint32 UniqueID;
    TWeakObjectPtr< ULevel > Level; // The level this bounds belongs to

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

UCLASS( config = Engine, defaultconfig )
class SVONAVIGATION_API USVONavigationSystem : public UEngineSubsystem
{
    GENERATED_BODY()

public:
    void OnNavigationVolumeAdded( const ASVONavigationVolume & volume );
    void OnNavigationVolumeRemoved( const ASVONavigationVolume & volume );
    void OnNavigationVolumeUpdated( const ASVONavigationVolume & volume );

private:

    void AddNavigationVolumeUpdateRequest( const FSVONavigationBoundsUpdateRequest & update_request );

    /*
    The minimum size of a leaf voxel in the X, Y and Z dimensions.
    */
    UPROPERTY( EditAnywhere, meta = ( ClampMin = "0.000001" ), DisplayName = "Minimum Voxel Size", Category = "SVO Navigation|Volume" )
    float VoxelSize = 200.0f;

    /*
    Which collision channel to use for object tracing during octree generation
    */
    UPROPERTY( EditAnywhere, Category = "SVO Navigation|Generation" )
    TEnumAsByte< ECollisionChannel > CollisionChannel;

    /*
    The minimum distance away from any object traces to apply during octree generation
    */
    UPROPERTY( EditAnywhere, Category = "SVO Navigation|Generation" )
    float Clearance = 0.0f;

    TArray< FSVONavigationBoundsUpdateRequest > PendingNavigationVolumeUpdateRequests;

    /** All areas where we build/have navigation */
    TSet< FSVONavigationBounds > RegisteredNavigationBounds;
};

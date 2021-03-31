#pragma once

#include <CoreMinimal.h>
#include <Engine/DeveloperSettings.h>

#include "SVONavigationSettings.generated.h"

DECLARE_MULTICAST_DELEGATE( FOnPropertyChangedDelegate );

UCLASS( config = Engine, defaultconfig )
class SVONAVIGATION_API USVONavigationSettings final : public UDeveloperSettings
{
    GENERATED_BODY()

public:

#if WITH_EDITOR
    FOnPropertyChangedDelegate OnPropertyChangedDelegate;

    void PostEditChangeProperty( FPropertyChangedEvent & property_changed_event ) override;
#endif

    /*
    The minimum size of a leaf voxel in the X, Y and Z dimensions.
    */
    UPROPERTY( config, EditAnywhere, meta = ( ClampMin = "0.000001" ), DisplayName = "Minimum Voxel Size", Category = "SVO Navigation|Volume" )
    float VoxelSize = 200.0f;

    /*
    Which collision channel to use for object tracing during octree generation
    */
    UPROPERTY( config, EditAnywhere, Category = "SVO Navigation|Generation" )
    TEnumAsByte< ECollisionChannel > CollisionChannel;

    /*
    The minimum distance away from any object traces to apply during octree generation
    */
    UPROPERTY( config, EditAnywhere, Category = "SVO Navigation|Generation" )
    float Clearance = 0.0f;
};

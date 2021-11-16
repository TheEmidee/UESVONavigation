#pragma once

#include <CoreMinimal.h>
#include <GameFramework/Actor.h>

#include "SVORaycasterTest.generated.h"

class USphereComponent;
class USVORaycaster;

UCLASS( hidecategories = ( Object, Actor, Input, Rendering, Replication, LOD, Cooking, Physics, Collision, Lighting, VirtualTexture, HLOD ), showcategories = ( "Input|MouseInput", "Input|TouchInput" ) )
class SVONAVIGATION_API ASVORaycasterTest final : public AActor
{
    GENERATED_BODY()

public:
    ASVORaycasterTest();

private:

    UFUNCTION( CallInEditor )
    void DoRaycast();

    UPROPERTY( VisibleAnywhere, BlueprintReadOnly, meta = ( AllowPrivateAccess = "true" ) )
    USphereComponent * SphereComponent;

    UPROPERTY( Instanced, EditAnywhere )
    USVORaycaster * Raycaster;

    UPROPERTY( EditInstanceOnly )
    AActor * OtherActor;

    UPROPERTY( EditAnywhere )
    FNavAgentProperties NavAgentProperties;
};

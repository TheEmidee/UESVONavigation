#pragma once

#include <CoreMinimal.h>
#include <GameFramework/Actor.h>

#include "SVORaycasterTest.generated.h"

class USVORaycaster;
UCLASS()
class SVONAVIGATION_API ASVORaycasterTest final : public AActor
{
    GENERATED_BODY()

public:
    ASVORaycasterTest();

private:

    UFUNCTION( CallInEditor )
    void DoRaycast();

    UPROPERTY( Instanced )
    USVORaycaster * Raycaster;

    UPROPERTY( EditInstanceOnly )
    AActor * OtherActor;
};

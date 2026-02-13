#pragma once

#include <CoreMinimal.h>
#include <Kismet/BlueprintFunctionLibrary.h>

#include "SVOBlueprintFunctionLibrary.generated.h"

UCLASS()
class SVONAVIGATION_API USVOBlueprintFunctionLibrary final : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()

    UFUNCTION( BlueprintPure, Category = "SVO Navigation" )
    static bool IsActorInNavigableArea( const AActor * actor );
};

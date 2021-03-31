#pragma once

#include <CoreMinimal.h>
#include <GameFramework/Volume.h>

#include "SVONavigationVolume.generated.h"

UCLASS()
class SVONAVIGATION_API ASVONavigationVolume : public AVolume
{
    GENERATED_BODY()

public:

    ASVONavigationVolume();

#if WITH_EDITOR
    void PostEditUndo() override;
    void PostEditChangeProperty( FPropertyChangedEvent & PropertyChangedEvent ) override;
#endif

    void PostRegisterAllComponents() override;
    void PostUnregisterAllComponents() override;
};

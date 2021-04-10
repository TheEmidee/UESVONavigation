#pragma once

#include <CoreMinimal.h>
#include <Engine/DeveloperSettings.h>

#include "SVONavigationSettings.generated.h"

UCLASS( config = Engine, defaultconfig )
class SVONAVIGATION_API USVONavigationSettings final : public UDeveloperSettings
{
    GENERATED_BODY()

public:

    USVONavigationSettings();

    UPROPERTY( config, EditAnywhere, Category = "SVO Navigation" )
    uint8 ShouldDiscardSubLevelNavigationData : 1;

    UPROPERTY( config, EditAnywhere, Category = "SVO Navigation" )
    uint8 NavigationAutoUpdateEnabled : 1;
};

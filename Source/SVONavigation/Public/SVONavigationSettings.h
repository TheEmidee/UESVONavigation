#pragma once

#include <CoreMinimal.h>
#include <Engine/DeveloperSettings.h>

#include "SVONavigationSettings.generated.h"

class USVORayCaster;
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

    // The algorithm to use to detect if there's a direct line of sight between the start of tha path and the target
    // If there's a direct LoS, the generated path will be a straight line from start to target.
    // Otherwise the pathfinding algorithm will be executed.
    // If that option is not set, the pathfinding will be always executed.
    UPROPERTY( config, EditAnywhere, Category = "PathFinding" )
    TSubclassOf< USVORayCaster > DefaultLineOfSightClass;
};

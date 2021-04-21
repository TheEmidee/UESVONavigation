#pragma once

#include <CoreMinimal.h>


#include "SVOPathFinder.h"
#include "SVOPathfindingRenderingComponent.h"
#include "Components/BillboardComponent.h"

#include <GameFramework/Actor.h>

#include "SVOPathFinderTest.generated.h"

class FSVOPathFinder;
class USphereComponent;
class USVOPathFindingRenderingComponent;

UCLASS()
class SVONAVIGATION_API ASVOPathFinderTest : public AActor
{
    GENERATED_BODY()

public:
    ASVOPathFinderTest();

    void PostEditChangeProperty( FPropertyChangedEvent & property_changed_event ) override;

    FVector GetStartLocation() const;
    FVector GetEndLocation() const;
    const TArray< FSVOPathFinderDebugStep > GetDebugSteps() const;

private:
    void UpdateDrawing();

#if WITH_EDITOR
    static void OnEditorSelectionChanged( UObject * new_selection );
#endif

    UFUNCTION( CallInEditor )
    void DoPathFinding();

    UFUNCTION( CallInEditor )
    void StepPathFinder();

    UPROPERTY( VisibleAnywhere, BlueprintReadOnly, meta = ( AllowPrivateAccess = "true" ) )
    UBillboardComponent * StartLocationComponent;

    UPROPERTY( VisibleAnywhere, BlueprintReadOnly, meta = ( AllowPrivateAccess = "true" ) )
    UBillboardComponent * EndLocationComponent;

#if WITH_EDITORONLY_DATA
    UPROPERTY( Transient )
    USVOPathFindingRenderingComponent * RenderingComponent;
#endif

    UPROPERTY( EditAnywhere )
	FNavAgentProperties NavAgentProperties;

    UPROPERTY( EditAnywhere )
    TSubclassOf< UNavigationQueryFilter > NavigationQueryFilter;

    TSharedPtr< FSVOPathFinder > PathFinder;
    FNavigationPath NavigationPath;
    TArray< FSVOPathFinderDebugStep > DebugSteps;
    uint8 bFoundPath : 1;
};

FORCEINLINE FVector ASVOPathFinderTest::GetStartLocation() const
{
    return StartLocationComponent->GetComponentLocation();
}

FORCEINLINE FVector ASVOPathFinderTest::GetEndLocation() const
{
    return EndLocationComponent->GetComponentLocation();
}

FORCEINLINE const TArray< FSVOPathFinderDebugStep > ASVOPathFinderTest::GetDebugSteps() const
{
    return DebugSteps;
}
#pragma once

#include <CoreMinimal.h>

#include "Components/BillboardComponent.h"

#include <GameFramework/Actor.h>

#include "SVOPathFinderTest.generated.h"

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

private:
    void UpdateDrawing();

#if WITH_EDITOR
    static void OnEditorSelectionChanged( UObject * new_selection );
#endif

    UFUNCTION( CallInEditor )
    void DoPathFinding();

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
};

FORCEINLINE FVector ASVOPathFinderTest::GetStartLocation() const
{
    return StartLocationComponent->GetComponentLocation();
}

FORCEINLINE FVector ASVOPathFinderTest::GetEndLocation() const
{
    return EndLocationComponent->GetComponentLocation();
}
#pragma once

#include <CoreMinimal.h>

#include "SVOPathFinder.h"
#include "SVOPathfindingRenderingComponent.h"

#include <Components/BillboardComponent.h>
#include <GameFramework/Actor.h>

#include "SVOPathFinderTest.generated.h"

class FSVOPathFinder;
class USphereComponent;

UCLASS()
class SVONAVIGATION_API ASVOPathFinderTest : public AActor
{
    GENERATED_BODY()

public:
    ASVOPathFinderTest();

    void PostEditChangeProperty( FPropertyChangedEvent & property_changed_event ) override;

    FVector GetStartLocation() const;
    FVector GetEndLocation() const;
    const FSVOPathFinderDebugInfos & GetPathFinderDebugInfos() const;
    const FSVOPathRenderingDebugDrawOptions & GetDebugDrawOptions() const;

private:
    void UpdateDrawing();

#if WITH_EDITOR
    static void OnEditorSelectionChanged( UObject * new_selection );
#endif

    UFUNCTION( CallInEditor )
    void InitPathFinding();

    UFUNCTION( CallInEditor )
    void Step();

    UFUNCTION( CallInEditor )
    void AutoComplete();

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

    UPROPERTY( EditAnywhere )
    FSVOPathRenderingDebugDrawOptions DebugDrawOptions;

    UPROPERTY( EditAnywhere )
    float AutoStepTimer;    

    TSharedPtr< FSVOPathFinder > PathFinder;
    FNavigationPath NavigationPath;

    UPROPERTY( VisibleInstanceOnly, AdvancedDisplay )
    FSVOPathFinderDebugInfos PathFinderDebugInfos;
    
    uint8 bFoundPath : 1;
    uint8 bAutoComplete : 1;
    FTimerHandle AutoCompleteTimerHandle;
};

FORCEINLINE FVector ASVOPathFinderTest::GetStartLocation() const
{
    return StartLocationComponent->GetComponentLocation();
}

FORCEINLINE FVector ASVOPathFinderTest::GetEndLocation() const
{
    return EndLocationComponent->GetComponentLocation();
}

FORCEINLINE const FSVOPathFinderDebugInfos & ASVOPathFinderTest::GetPathFinderDebugInfos() const
{
    return PathFinderDebugInfos;
}

FORCEINLINE const FSVOPathRenderingDebugDrawOptions & ASVOPathFinderTest::GetDebugDrawOptions() const
{
    return DebugDrawOptions;
}
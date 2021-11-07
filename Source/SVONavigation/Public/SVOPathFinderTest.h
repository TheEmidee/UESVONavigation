#pragma once

#include "SVOPathFindingAlgorithm.h"

#include <CoreMinimal.h>
#include <DebugRenderSceneProxy.h>
#include <GameFramework/Actor.h>

#include "SVOPathFinderTest.generated.h"

class FSVOPathFinder;
class USphereComponent;

class ASVOPathFinderTest;
class USVOPathFindingRenderingComponent;

USTRUCT()
struct SVONAVIGATION_API FSVOPathRenderingDebugDrawOptions
{
    GENERATED_USTRUCT_BODY()

    FSVOPathRenderingDebugDrawOptions() :
        bDrawOnlyWhenSelected( false ),
        bDrawLastProcessedNode( true ),
        bDrawLastProcessedNodeCost( false ),
        bDrawLastProcessedNeighbors( true ),
        bDrawNeighborsCost( false ),
        bDrawBestPath( true )
    {}

    UPROPERTY( EditAnywhere )
    uint8 bDrawOnlyWhenSelected : 1;

    UPROPERTY( EditAnywhere )
    uint8 bDrawLastProcessedNode : 1;

    UPROPERTY( EditAnywhere, meta = ( EditCondition = "bDrawLastProcessedNode" ) )
    uint8 bDrawLastProcessedNodeCost : 1;

    UPROPERTY( EditAnywhere )
    uint8 bDrawLastProcessedNeighbors : 1;

    UPROPERTY( EditAnywhere, meta = ( EditCondition = "bDrawNeighborsCost" ) )
    uint8 bDrawNeighborsCost : 1;

    UPROPERTY( EditAnywhere )
    uint8 bDrawBestPath : 1;
};

struct SVONAVIGATION_API FSVOPathFindingSceneProxyData final : public TSharedFromThis< FSVOPathFindingSceneProxyData, ESPMode::ThreadSafe >
{
    void GatherData( const ASVOPathFinderTest & path_finder_test );

    FVector StartLocation;
    FVector EndLocation;
    FSVOPathFinderDebugInfos DebugInfos;
    TOptional< EGraphAStarResult > PathFindingResult;
    TSharedPtr< FSVOPathFindingAlgorithmStepper > Stepper;
};

class SVONAVIGATION_API FSVOPathFindingSceneProxy final : public FDebugRenderSceneProxy
{
    friend class FSVOPathFindingRenderingDebugDrawDelegateHelper;

public:
    FSVOPathFindingSceneProxy( const UPrimitiveComponent & component, const FSVOPathFindingSceneProxyData & proxy_data );

    SIZE_T GetTypeHash() const override;
    FPrimitiveViewRelevance GetViewRelevance( const FSceneView * view ) const override;
    void GetDynamicMeshElements( const TArray<const FSceneView *> & views, const FSceneViewFamily & view_family, uint32 visibility_map, FMeshElementCollector & collector ) const override;

private:
    bool SafeIsActorSelected() const;

    AActor * ActorOwner;
    FSVOPathRenderingDebugDrawOptions DebugDrawOptions;
    TWeakObjectPtr< ASVOPathFinderTest > PathFinderTest;
    TWeakObjectPtr< USVOPathFindingRenderingComponent > RenderingComponent;
    TArray< TPair< FVector, FVector > > ArrowHeadLocations;
};

class FSVOPathFindingRenderingDebugDrawDelegateHelper final : public FDebugDrawDelegateHelper
{
    typedef FDebugDrawDelegateHelper Super;

public:
    FSVOPathFindingRenderingDebugDrawDelegateHelper() :
        ActorOwner( nullptr )
    {
    }

    void InitDelegateHelper( const FDebugRenderSceneProxy * InSceneProxy ) override
    {
        check( 0 );
    }

    void InitDelegateHelper( const FSVOPathFindingSceneProxy & InSceneProxy );

protected:
    SVONAVIGATION_API void DrawDebugLabels( UCanvas * Canvas, APlayerController * ) override;

private:
    // can be 0
    AActor * ActorOwner;
    FSVOPathRenderingDebugDrawOptions DebugDrawOptions;
};

UCLASS()
class SVONAVIGATION_API USVOPathFindingRenderingComponent final : public UPrimitiveComponent
{
    GENERATED_BODY()

public:
    USVOPathFindingRenderingComponent();

    ASVOPathFinderTest * GetPathFinderTest() const;
    FPrimitiveSceneProxy * CreateSceneProxy() override;

    void CreateRenderState_Concurrent( FRegisterComponentContext * Context ) override;
    void DestroyRenderState_Concurrent() override;
    FBoxSphereBounds CalcBounds( const FTransform & local_to_world ) const override;

private:
    void GatherData( FSVOPathFindingSceneProxyData & proxy_data, const ASVOPathFinderTest & path_finder_test );

    FSVOPathFindingRenderingDebugDrawDelegateHelper RenderingDebugDrawDelegateHelper;
};

FORCEINLINE ASVOPathFinderTest * USVOPathFindingRenderingComponent::GetPathFinderTest() const
{
    return Cast< ASVOPathFinderTest >( GetOwner() );
}

UCLASS( hidecategories = ( Object, Actor, Input, Rendering, Replication, LOD, Cooking, Physics, Collision, Lighting, VirtualTexture, HLOD ), showcategories = ( "Input|MouseInput", "Input|TouchInput" ), Blueprintable )
class SVONAVIGATION_API ASVOPathFinderTest final : public AActor
{
    GENERATED_BODY()

public:
    ASVOPathFinderTest();

#if WITH_EDITOR
    void PreEditChange( FProperty * property_about_to_change ) override;
    void PostEditChangeProperty( FPropertyChangedEvent & property_changed_event ) override;
    void PostEditMove( bool is_finished ) override;
#endif

    FVector GetStartLocation() const;
    FVector GetEndLocation() const;
    TSharedPtr< FSVOPathFindingAlgorithmStepper > GetStepper() const;
    const FSVOPathFinderDebugInfos & GetPathFinderDebugInfos() const;
    const FSVOPathRenderingDebugDrawOptions & GetDebugDrawOptions() const;
    ESVOPathFindingAlgorithmStepperStatus GetStepperLastStatus() const;
    EGraphAStarResult GetPathFindingResult() const;
    void BeginDestroy() override;

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
    void AutoCompleteStepByStep();

    UFUNCTION( CallInEditor )
    void AutoCompleteUntilNextNode();

    UFUNCTION( CallInEditor )
    void AutoCompleteInstantly();

    UFUNCTION( CallInEditor )
    void PauseAutoCompletion();

    UPROPERTY( VisibleAnywhere, BlueprintReadOnly, meta = ( AllowPrivateAccess = "true" ) )
    USphereComponent * SphereComponent;

#if WITH_EDITORONLY_DATA
    UPROPERTY( Transient )
    USVOPathFindingRenderingComponent * RenderingComponent;
#endif

    UPROPERTY( EditAnywhere )
    uint8 bUpdatePathAfterMoving : 1;

    UPROPERTY( EditAnywhere )
    FNavAgentProperties NavAgentProperties;

    UPROPERTY( EditAnywhere )
    TSubclassOf< UNavigationQueryFilter > NavigationQueryFilter;

    UPROPERTY( EditAnywhere )
    FSVOPathRenderingDebugDrawOptions DebugDrawOptions;

    TSharedPtr< FSVOPathFindingAlgorithmStepper > Stepper;

    UPROPERTY( EditAnywhere )
    float AutoStepTimer;

    UPROPERTY( EditInstanceOnly )
    ASVOPathFinderTest * OtherActor;

    FNavigationPath NavigationPath;

    UPROPERTY( VisibleInstanceOnly, AdvancedDisplay )
    FSVOPathFinderDebugInfos PathFinderDebugInfos;

    uint8 bAutoComplete : 1;
    FTimerHandle AutoCompleteTimerHandle;
    ESVOPathFindingAlgorithmStepperStatus LastStatus;
    EGraphAStarResult PathFindingResult;
};

FORCEINLINE FVector ASVOPathFinderTest::GetStartLocation() const
{
    return GetActorLocation();
}

FORCEINLINE FVector ASVOPathFinderTest::GetEndLocation() const
{
    return OtherActor != nullptr
               ? OtherActor->GetActorLocation()
               : FVector::ZeroVector;
}

FORCEINLINE TSharedPtr< FSVOPathFindingAlgorithmStepper > ASVOPathFinderTest::GetStepper() const
{
    return Stepper;
}

FORCEINLINE const FSVOPathFinderDebugInfos & ASVOPathFinderTest::GetPathFinderDebugInfos() const
{
    return PathFinderDebugInfos;
}

FORCEINLINE const FSVOPathRenderingDebugDrawOptions & ASVOPathFinderTest::GetDebugDrawOptions() const
{
    return DebugDrawOptions;
}

FORCEINLINE ESVOPathFindingAlgorithmStepperStatus ASVOPathFinderTest::GetStepperLastStatus() const
{
    return LastStatus;
}

FORCEINLINE EGraphAStarResult ASVOPathFinderTest::GetPathFindingResult() const
{
    return PathFindingResult;
}
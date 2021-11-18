#pragma once

#include "SVORayCaster.h"

#include <CoreMinimal.h>
#include <DebugRenderSceneProxy.h>
#include <GameFramework/Actor.h>

#include "SVORaycasterTest.generated.h"

class USVORayCasterRenderingComponent;
class ASVORaycasterTest;
class USphereComponent;
class USVORayCaster;

USTRUCT()
struct SVONAVIGATION_API FSVORayCasterDebugDrawOptions
{
    GENERATED_USTRUCT_BODY()

    UPROPERTY( EditAnywhere )
    uint8 bEnableDebugDraw : 1;

    UPROPERTY( EditAnywhere )
    uint8 bDrawLayerNodes : 1;

    UPROPERTY( EditAnywhere, meta = ( EditCondition = "bDrawLayerNodes", ClampMin = "0", UIMin = "0" ) )
    uint8 LayerIndexToDraw;

    UPROPERTY( EditAnywhere )
    uint8 bDrawSubNodes : 1;
};

struct SVONAVIGATION_API FSVORayCasterSceneProxyData final : public TSharedFromThis< FSVORayCasterSceneProxyData, ESPMode::ThreadSafe >
{
    void GatherData( const ASVORaycasterTest & ray_caster_test );

    FSVORayCasterDebugInfos DebugInfos;
};

class SVONAVIGATION_API FSVORayCasterSceneProxy final : public FDebugRenderSceneProxy
{
public:
    FSVORayCasterSceneProxy( const UPrimitiveComponent & component, const FSVORayCasterSceneProxyData & proxy_data );

    SIZE_T GetTypeHash() const override;
    FPrimitiveViewRelevance GetViewRelevance( const FSceneView * view ) const override;

private:
    TWeakObjectPtr< ASVORaycasterTest > RayCasterTest;
};

UCLASS()
class SVONAVIGATION_API USVORayCasterRenderingComponent final : public UPrimitiveComponent
{
    GENERATED_BODY()

public:
    USVORayCasterRenderingComponent() = default;

    ASVORaycasterTest * GetRayCasterTest() const;
    FPrimitiveSceneProxy * CreateSceneProxy() override;

    FBoxSphereBounds CalcBounds( const FTransform & local_to_world ) const override;
};

UCLASS( hidecategories = ( Object, Actor, Input, Rendering, Replication, LOD, Cooking, Physics, Collision, Lighting, VirtualTexture, HLOD ), showcategories = ( "Input|MouseInput", "Input|TouchInput" ) )
class SVONAVIGATION_API ASVORaycasterTest final : public AActor
{
    GENERATED_BODY()

public:
    ASVORaycasterTest();

    const FSVORayCasterDebugInfos & GetDebugInfos() const;
    const FSVORayCasterDebugDrawOptions & GetDebugDrawOptions() const;

    FBoxSphereBounds GetBoundingBoxContainingOtherActorAndMe() const;

#if WITH_EDITOR
    void PreEditChange( FProperty * property_about_to_change ) override;
    void PostEditChangeProperty( FPropertyChangedEvent & property_changed_event ) override;
    void PostEditMove( bool is_finished ) override;
#endif

    void BeginDestroy() override;

private:
    void UpdateDrawing();

    UFUNCTION( CallInEditor )
    void DoRaycast();

    UPROPERTY( VisibleAnywhere, BlueprintReadOnly, meta = ( AllowPrivateAccess = "true" ) )
    USphereComponent * SphereComponent;

#if WITH_EDITORONLY_DATA
    UPROPERTY( Transient )
    USVORayCasterRenderingComponent * RenderingComponent;
#endif

    UPROPERTY( Instanced, EditAnywhere )
    USVORayCaster * Raycaster;

    UPROPERTY( EditInstanceOnly )
    ASVORaycasterTest * OtherActor;

    UPROPERTY( EditAnywhere )
    FNavAgentProperties NavAgentProperties;

    UPROPERTY( EditAnywhere )
    uint8 bUpdatePathAfterMoving : 1;

    UPROPERTY( EditAnywhere )
    FSVORayCasterDebugDrawOptions DebugDrawOptions;

    FSVORayCasterDebugInfos RayCasterDebugInfos;
};

FORCEINLINE const FSVORayCasterDebugInfos & ASVORaycasterTest::GetDebugInfos() const
{
    return RayCasterDebugInfos;
}

FORCEINLINE const FSVORayCasterDebugDrawOptions & ASVORaycasterTest::GetDebugDrawOptions() const
{
    return DebugDrawOptions;
}
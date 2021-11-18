#pragma once

#include <CoreMinimal.h>
#include <DebugRenderSceneProxy.h>
#include <GameFramework/Actor.h>

#include "SVORaycasterTest.generated.h"

class USVORayCasterRenderingComponent;
class ASVORaycasterTest;
class USphereComponent;
class USVORaycaster;

struct SVONAVIGATION_API FSVORayCasterSceneProxyData final : public TSharedFromThis< FSVORayCasterSceneProxyData, ESPMode::ThreadSafe >
{
    void GatherData( const ASVORaycasterTest & ray_caster_test );
};

class SVONAVIGATION_API FSVORayCasterSceneProxy final : public FDebugRenderSceneProxy
{
public:
    FSVORayCasterSceneProxy( const UPrimitiveComponent & component, const FSVORayCasterSceneProxyData & proxy_data );

    SIZE_T GetTypeHash() const override;
    FPrimitiveViewRelevance GetViewRelevance( const FSceneView * view ) const override;

private:
    bool SafeIsActorSelected() const;

    AActor * ActorOwner;
    TWeakObjectPtr< ASVORaycasterTest > RayCasterTest;
    TWeakObjectPtr< USVORayCasterRenderingComponent > RenderingComponent;
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
    USVORaycaster * Raycaster;

    UPROPERTY( EditInstanceOnly )
    ASVORaycasterTest * OtherActor;

    UPROPERTY( EditAnywhere )
    FNavAgentProperties NavAgentProperties;

    UPROPERTY( EditAnywhere )
    uint8 bUpdatePathAfterMoving : 1;
};

#pragma once

#include "SVONavigationTypes.h"

#include <Components/PrimitiveComponent.h>
#include <CoreMinimal.h>
#include <DebugRenderSceneProxy.h>
#include <Math/GenericOctree.h>

#include "SVONavDataRenderingComponent.generated.h"

class ASVONavigationData;
class USVONavDataRenderingComponent;

struct FDebugText
{
    FVector Location;
    FString Text;

    FDebugText()
    {}
    FDebugText( const FVector & InLocation, const FString & InText ) :
        Location( InLocation ),
        Text( InText )
    {}
};

class SVONAVIGATION_API FSVONavigationMeshSceneProxy final : public FDebugRenderSceneProxy
{
public:
    explicit FSVONavigationMeshSceneProxy( const UPrimitiveComponent * component );
    virtual ~FSVONavigationMeshSceneProxy() override;

    SIZE_T GetTypeHash() const override;

protected:
    FPrimitiveViewRelevance GetViewRelevance( const FSceneView * view ) const override;
    TWeakObjectPtr< USVONavDataRenderingComponent > RenderingComponent;
};

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
class FSVODebugDrawDelegateHelper final : public FDebugDrawDelegateHelper
{
    typedef FDebugDrawDelegateHelper Super;

public:
    FSVODebugDrawDelegateHelper() = default;

    void InitDelegateHelper( const FDebugRenderSceneProxy * InSceneProxy ) override
    {
        check( 0 );
    }

    void InitDelegateHelper( const FSVONavigationMeshSceneProxy * scene_proxy )
    {
        Super::InitDelegateHelper( scene_proxy );

        DebugLabels.Reset();
        //DebugLabels.Append( scene_proxy->GetDebugTexts() );
    }

    SVONAVIGATION_API void RegisterDebugDrawDelgate() override;
    SVONAVIGATION_API void UnregisterDebugDrawDelgate() override;

protected:
    SVONAVIGATION_API void DrawDebugLabels( UCanvas * Canvas, APlayerController * ) override;

private:
    TArray< FDebugText > DebugLabels;
};
#endif

UCLASS()
class SVONAVIGATION_API USVONavDataRenderingComponent final : public UPrimitiveComponent
{
    GENERATED_BODY()

public:
    USVONavDataRenderingComponent();

    void ForceUpdate();
    bool UpdateIsForced() const;

    FPrimitiveSceneProxy * CreateSceneProxy() override;
    FBoxSphereBounds CalcBounds( const FTransform & LocalToWorld ) const override;

    void CreateRenderState_Concurrent( FRegisterComponentContext * context ) override;
    void DestroyRenderState_Concurrent() override;

    static bool IsNavigationShowFlagSet( const UWorld * world );

private:
    uint8 bForcesUpdate : 1;

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
    FSVODebugDrawDelegateHelper DebugDrawDelegateManager;
#endif
};

FORCEINLINE void USVONavDataRenderingComponent::ForceUpdate()
{
    bForcesUpdate = true;
}

FORCEINLINE bool USVONavDataRenderingComponent::UpdateIsForced() const
{
    return bForcesUpdate;
}
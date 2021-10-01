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

struct SVONAVIGATION_API FSVONavigationSceneProxyData : public TSharedFromThis< FSVONavigationSceneProxyData, ESPMode::ThreadSafe >
{
    const TArray< FBoxCenterAndExtent > & GetOctreeBounds() const;
    const TArray< FBoxCenterAndExtent > & GetLayers() const;
    const TArray< FBoxCenterAndExtent > & GetLeaves() const;
    const TArray< FBoxCenterAndExtent > & GetOccludedLeaves() const;
    const TArray< FDebugRenderSceneProxy::FDebugLine > & GetLinks() const;
    const FSVONavigationBoundsDataDebugInfos & GetDebugInfos() const;
    const TArray< FDebugText > & GetDebugTexts() const;

    void Reset();
    void Serialize( FArchive & archive );
    uint32 GetAllocatedSize() const;
    void GatherData( const ASVONavigationData & navigation_data );

private:
    TArray< FBoxCenterAndExtent > OctreeBounds;
    TArray< FBoxCenterAndExtent > Layers;
    TArray< FBoxCenterAndExtent > Leaves;
    TArray< FBoxCenterAndExtent > OccludedLeaves;
    TArray< FDebugRenderSceneProxy::FDebugLine > Links;
    TArray< FDebugText > DebugTexts;

    FSVONavigationBoundsDataDebugInfos DebugInfos;
};

FORCEINLINE const TArray< FBoxCenterAndExtent > & FSVONavigationSceneProxyData::GetOctreeBounds() const
{
    return OctreeBounds;
}

FORCEINLINE const TArray< FBoxCenterAndExtent > & FSVONavigationSceneProxyData::GetLayers() const
{
    return Layers;
}

FORCEINLINE const TArray< FBoxCenterAndExtent > & FSVONavigationSceneProxyData::GetLeaves() const
{
    return Leaves;
}

FORCEINLINE const TArray< FBoxCenterAndExtent > & FSVONavigationSceneProxyData::GetOccludedLeaves() const
{
    return OccludedLeaves;
}

FORCEINLINE const TArray< FDebugRenderSceneProxy::FDebugLine > & FSVONavigationSceneProxyData::GetLinks() const
{
    return Links;
}

FORCEINLINE const FSVONavigationBoundsDataDebugInfos & FSVONavigationSceneProxyData::GetDebugInfos() const
{
    return DebugInfos;
}

FORCEINLINE const TArray< FDebugText > & FSVONavigationSceneProxyData::GetDebugTexts() const
{
    return DebugTexts;
}

class SVONAVIGATION_API FSVONavigationMeshSceneProxy final : public FDebugRenderSceneProxy
{
public:
    SIZE_T GetTypeHash() const override;
    const FSVONavigationSceneProxyData & GetProxyData() const;

    FSVONavigationMeshSceneProxy( const UPrimitiveComponent * component, FSVONavigationSceneProxyData * proxy_data /* , bool ForceToRender = false */ );
    virtual ~FSVONavigationMeshSceneProxy() override;

    void GetDynamicMeshElements( const TArray< const FSceneView * > & views, const FSceneViewFamily & view_family, uint32 visibility_map, FMeshElementCollector & collector ) const override;

protected:
    FPrimitiveViewRelevance GetViewRelevance( const FSceneView * view ) const override;

private:
    FSVONavigationSceneProxyData ProxyData;
    TWeakObjectPtr< USVONavDataRenderingComponent > RenderingComponent;
};

FORCEINLINE const FSVONavigationSceneProxyData & FSVONavigationMeshSceneProxy::GetProxyData() const
{
    return ProxyData;
}

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
class FSVODebugDrawDelegateHelper final : public FDebugDrawDelegateHelper
{
    typedef FDebugDrawDelegateHelper Super;

public:
    FSVODebugDrawDelegateHelper() /*:
        bForceRendering( false ),
        bNeedsNewData( false )*/
    {
    }

    virtual void InitDelegateHelper( const FDebugRenderSceneProxy * InSceneProxy ) override
    {
        check( 0 );
    }

    void InitDelegateHelper( const FSVONavigationMeshSceneProxy * InSceneProxy )
    {
        Super::InitDelegateHelper( InSceneProxy );

        DebugLabels.Reset();
        DebugLabels.Append( InSceneProxy->GetProxyData().GetDebugTexts() );
        /*bForceRendering = InSceneProxy->bForceRendering;
        bNeedsNewData = InSceneProxy->ProxyData.bNeedsNewData;*/
    }

    SVONAVIGATION_API void RegisterDebugDrawDelgate() override;
    SVONAVIGATION_API void UnregisterDebugDrawDelgate() override;

protected:
    SVONAVIGATION_API void DrawDebugLabels( UCanvas * Canvas, APlayerController * ) override;

private:
    TArray< FDebugText > DebugLabels;
    /*uint32 bForceRendering : 1;
    uint32 bNeedsNewData : 1;*/
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
    void GatherData( FSVONavigationSceneProxyData & proxy_data, const ASVONavigationData & navigation_data ) const;

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
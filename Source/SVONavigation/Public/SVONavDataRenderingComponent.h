#pragma once

#include <Components/PrimitiveComponent.h>
#include <CoreMinimal.h>
#include <DebugRenderSceneProxy.h>

#include "SVONavDataRenderingComponent.generated.h"

class ASVONavigationData;
class USVONavDataRenderingComponent;

struct SVONAVIGATION_API FSVONavigationSceneProxyData : public TSharedFromThis< FSVONavigationSceneProxyData, ESPMode::ThreadSafe >
{
    FSVONavigationSceneProxyData() :
        bDataGathered( false ),
        bNeedsNewData( true )
    {}

    const TArray< FBoxCenterAndExtent > & GetOctreeBounds() const;
    const TArray< FBoxCenterAndExtent > & GetLayers() const;
    const TArray< FBoxCenterAndExtent > & GetLeaves() const;
    const TArray< FBoxCenterAndExtent > & GetOccludedLeaves() const;

    void Reset();
    void Serialize( FArchive & Ar );
    uint32 GetAllocatedSize() const;
    void GatherData( const ASVONavigationData * navigation_data, int32 debug_draw_flags );

private:
    TArray< FBoxCenterAndExtent > OctreeBounds;
    TArray< FBoxCenterAndExtent > Layers;
    TArray< FBoxCenterAndExtent > Leaves;
    TArray< FBoxCenterAndExtent > OccludedLeaves;

    FBox Bounds;
    uint32 bDataGathered : 1;
    uint32 bNeedsNewData : 1;
    int32 DebugDrawFlags;
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

class SVONAVIGATION_API FSVONavigationMeshSceneProxy final : public FDebugRenderSceneProxy
{
public:
    SIZE_T GetTypeHash() const override;

    FSVONavigationMeshSceneProxy( const UPrimitiveComponent * component, FSVONavigationSceneProxyData * proxy_data /* , bool ForceToRender = false */ );
    virtual ~FSVONavigationMeshSceneProxy();

    void GetDynamicMeshElements( const TArray< const FSceneView * > & views, const FSceneViewFamily & view_family, uint32 visibility_map, FMeshElementCollector & collector ) const override;

protected:
    FPrimitiveViewRelevance GetViewRelevance( const FSceneView * view ) const override;

private:
    FSVONavigationSceneProxyData ProxyData;
    TWeakObjectPtr< USVONavDataRenderingComponent > RenderingComponent;
};

UCLASS()
class SVONAVIGATION_API USVONavDataRenderingComponent : public UPrimitiveComponent
{
    GENERATED_BODY()

public:
    USVONavDataRenderingComponent();

    FPrimitiveSceneProxy * CreateSceneProxy() override;
    FBoxSphereBounds CalcBounds( const FTransform & LocalToWorld ) const override;

private:
    void GatherData( FSVONavigationSceneProxyData & proxy_data, const ASVONavigationData & navigation_data ) const;
};

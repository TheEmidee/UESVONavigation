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

    void Reset();
    void Serialize( FArchive & Ar );
    uint32 GetAllocatedSize() const;
    void GatherData( const ASVONavigationData * navigation_data );

private:
    TArray< FBoxCenterAndExtent > OctreeBounds;

    FBox Bounds;
    uint32 bDataGathered : 1;
    uint32 bNeedsNewData : 1;
};

FORCEINLINE const TArray< FBoxCenterAndExtent > & FSVONavigationSceneProxyData::GetOctreeBounds() const
{
    return OctreeBounds;
}

class SVONAVIGATION_API FSVONavigationMeshSceneProxy final : public FDebugRenderSceneProxy
{
public:
    SIZE_T GetTypeHash() const override;

    FSVONavigationMeshSceneProxy( const UPrimitiveComponent * InComponent, FSVONavigationSceneProxyData * InProxyData /* , bool ForceToRender = false */ );
    virtual ~FSVONavigationMeshSceneProxy();

    void GetDynamicMeshElements( const TArray< const FSceneView * > & Views, const FSceneViewFamily & ViewFamily, uint32 VisibilityMap, FMeshElementCollector & Collector ) const override;

protected:
    FPrimitiveViewRelevance GetViewRelevance( const FSceneView * View ) const override;

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

private:
    void GatherData( const ASVONavigationData & navigation_data, FSVONavigationSceneProxyData & OutProxyData ) const;
};

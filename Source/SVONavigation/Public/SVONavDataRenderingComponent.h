#pragma once

#include "SVONavigationTypes.h"

#include <Components/PrimitiveComponent.h>
#include <CoreMinimal.h>
#include <DebugRenderSceneProxy.h>
#include <Math/GenericOctree.h>

#include "SVONavDataRenderingComponent.generated.h"

class ASVONavigationData;
class USVONavDataRenderingComponent;

class SVONAVIGATION_API FSVONavigationMeshSceneProxy final : public FDebugRenderSceneProxy
{
public:
    friend class FSVODebugDrawDelegateHelper;

    explicit FSVONavigationMeshSceneProxy( const UPrimitiveComponent * component );
    virtual ~FSVONavigationMeshSceneProxy() override;

    SIZE_T GetTypeHash() const override;

protected:
    bool AddVoxelToBoxes( const FVector & voxel_location, const float node_extent, const bool is_occluded );
    void AddNodeTextInfos( const MortonCode node_morton_code, const LayerIndex node_layer_index, const FVector & node_position );
    void DrawNeighborInfos( const FSVOVolumeNavigationData & navigation_bounds_data, const FSVONodeAddress & node_address );

    FPrimitiveViewRelevance GetViewRelevance( const FSceneView * view ) const override;
    TWeakObjectPtr< USVONavDataRenderingComponent > RenderingComponent;
    TWeakObjectPtr< ASVONavigationData > NavigationData;
};

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
class FSVODebugDrawDelegateHelper final : public FDebugDrawDelegateHelper
{
    typedef FDebugDrawDelegateHelper Super;

public:
    FSVODebugDrawDelegateHelper() = default;

    void InitDelegateHelper( const FSVONavigationMeshSceneProxy * scene_proxy );

    SVONAVIGATION_API void RegisterDebugDrawDelegateInternal() override;
    SVONAVIGATION_API void UnregisterDebugDrawDelegate() override;

private:
    TWeakObjectPtr< ASVONavigationData > NavigationData;
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
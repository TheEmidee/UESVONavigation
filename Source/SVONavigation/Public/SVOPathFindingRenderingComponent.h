#pragma once

#include <Components/PrimitiveComponent.h>
#include <CoreMinimal.h>

#include "SVOPathFinder.h"

#include <DebugRenderSceneProxy.h>

#include "SVOPathfindingRenderingComponent.generated.h"

class USVOPathFindingRenderingComponent;
class ASVOPathFinderTest;

USTRUCT()
struct SVONAVIGATION_API FSVOPathRenderingDebugDrawOptions
{
    GENERATED_USTRUCT_BODY()
    
    FSVOPathRenderingDebugDrawOptions() :
        bDrawOnlyWhenSelected( false  ),
        bDrawCurrentCost( true ),
        bDrawNeighborsCost( true ),
        bDrawOnlyLastNeighborsCost( true )
    {}
    
    UPROPERTY( EditAnywhere )
    uint8 bDrawOnlyWhenSelected : 1;

    UPROPERTY( EditAnywhere )
    uint8 bDrawCurrentCost : 1;

    UPROPERTY( EditAnywhere )
    uint8 bDrawNeighborsCost : 1;

    UPROPERTY( EditAnywhere, meta = ( EditCondition = "bDrawNeighborsCost" ) )
    uint8 bDrawOnlyLastNeighborsCost : 1;
};

struct SVONAVIGATION_API FSVOPathFindingSceneProxyData : public TSharedFromThis< FSVOPathFindingSceneProxyData, ESPMode::ThreadSafe >
{
    FSVOPathFindingSceneProxyData()
    {}

    void GatherData( const ASVOPathFinderTest & path_finder_test );

    FVector StartLocation;
    FVector EndLocation;
    FSVOPathFinderDebugInfos DebugInfos;
};

class SVONAVIGATION_API FSVOPathFindingSceneProxy final : public FDebugRenderSceneProxy
{
    friend class FSVOPathFindingRenderingDebugDrawDelegateHelper;

public:
    FSVOPathFindingSceneProxy( const UPrimitiveComponent & component, const FSVOPathFindingSceneProxyData & proxy_data );

    SIZE_T GetTypeHash() const override;
    FPrimitiveViewRelevance GetViewRelevance( const FSceneView * view ) const override;

private:

    bool SafeIsActorSelected() const;

    AActor* ActorOwner;
	FSVOPathRenderingDebugDrawOptions DebugDrawOptions;
    TWeakObjectPtr< ASVOPathFinderTest > PathFinderTest;
    TWeakObjectPtr< USVOPathFindingRenderingComponent > RenderingComponent;
};

class FSVOPathFindingRenderingDebugDrawDelegateHelper : public FDebugDrawDelegateHelper
{
    typedef FDebugDrawDelegateHelper Super;

public:
    FSVOPathFindingRenderingDebugDrawDelegateHelper() :
        ActorOwner( nullptr )
    {
    }

    virtual void InitDelegateHelper( const FDebugRenderSceneProxy * InSceneProxy ) override
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
class SVONAVIGATION_API USVOPathFindingRenderingComponent : public UPrimitiveComponent
{
    GENERATED_BODY()

public:

    USVOPathFindingRenderingComponent();

    ASVOPathFinderTest * GetPathFinderTest() const;
    FPrimitiveSceneProxy * CreateSceneProxy() override;

    void CreateRenderState_Concurrent( FRegisterComponentContext * Context ) override;
    void DestroyRenderState_Concurrent() override;
    FBoxSphereBounds CalcBounds( const FTransform & LocalToWorld ) const override;

private:
    void GatherData( FSVOPathFindingSceneProxyData & proxy_data, const ASVOPathFinderTest & path_finder_test );

    FSVOPathFindingRenderingDebugDrawDelegateHelper RenderingDebugDrawDelegateHelper;
};

FORCEINLINE ASVOPathFinderTest * USVOPathFindingRenderingComponent::GetPathFinderTest() const
{
    return Cast< ASVOPathFinderTest >( GetOwner() );
}
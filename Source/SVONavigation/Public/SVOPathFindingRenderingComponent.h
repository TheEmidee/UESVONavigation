#pragma once

#include <Components/PrimitiveComponent.h>
#include <CoreMinimal.h>

#include "SVOPathFinder.h"

#include <DebugRenderSceneProxy.h>

#include "SVOPathfindingRenderingComponent.generated.h"

class USVOPathFindingRenderingComponent;
class ASVOPathFinderTest;

struct SVONAVIGATION_API FSVOPathFindingSceneProxyData : public TSharedFromThis< FSVOPathFindingSceneProxyData, ESPMode::ThreadSafe >
{
    FSVOPathFindingSceneProxyData()
    {}

    void GatherData( const ASVOPathFinderTest & path_finder_test );

    FVector StartLocation;
    FVector EndLocation;
    TArray< FSVOPathFinderDebugStep > DebugSteps;
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
	//const IEQSQueryResultSourceInterface* QueryDataSource;
	uint32 bDrawOnlyWhenSelected : 1;

    TWeakObjectPtr< USVOPathFindingRenderingComponent > RenderingComponent;
};

class FSVOPathFindingRenderingDebugDrawDelegateHelper : public FDebugDrawDelegateHelper
{
    typedef FDebugDrawDelegateHelper Super;

public:
    FSVOPathFindingRenderingDebugDrawDelegateHelper() :
        ActorOwner( nullptr ),
        //QueryDataSource( nullptr ),
        bDrawOnlyWhenSelected( false )
    {
    }

    virtual void InitDelegateHelper( const FDebugRenderSceneProxy * InSceneProxy ) override
    {
        check( 0 );
    }

    void InitDelegateHelper( const FSVOPathFindingSceneProxy * InSceneProxy )
    {
        Super::InitDelegateHelper( InSceneProxy );

        ActorOwner = InSceneProxy->ActorOwner;
        //QueryDataSource = InSceneProxy->QueryDataSource;
        bDrawOnlyWhenSelected = InSceneProxy->bDrawOnlyWhenSelected;
    }

protected:
    SVONAVIGATION_API void DrawDebugLabels( UCanvas * Canvas, APlayerController * ) override;

private:
    // can be 0
    AActor * ActorOwner;
    //const IEQSQueryResultSourceInterface * QueryDataSource;
    uint32 bDrawOnlyWhenSelected : 1;
};

UCLASS()
class SVONAVIGATION_API USVOPathFindingRenderingComponent : public UPrimitiveComponent
{
    GENERATED_BODY()

public:

    USVOPathFindingRenderingComponent();

    ASVOPathFinderTest * GetPathFinderTest() const;
    FPrimitiveSceneProxy * CreateSceneProxy() override;
    bool DrawOnlyWhenSelected() const;

    void CreateRenderState_Concurrent( FRegisterComponentContext * Context ) override;
    void DestroyRenderState_Concurrent() override;
    FBoxSphereBounds CalcBounds( const FTransform & LocalToWorld ) const override;

private:
    void GatherData( FSVOPathFindingSceneProxyData & proxy_data, const ASVOPathFinderTest & path_finder_test );

    FString DrawFlagName;
	uint32 bDrawOnlyWhenSelected : 1;
    FSVOPathFindingRenderingDebugDrawDelegateHelper RenderingDebugDrawDelegateHelper;
};

FORCEINLINE bool USVOPathFindingRenderingComponent::DrawOnlyWhenSelected() const
{
    return bDrawOnlyWhenSelected;
}

FORCEINLINE ASVOPathFinderTest * USVOPathFindingRenderingComponent::GetPathFinderTest() const
{
    return Cast< ASVOPathFinderTest >( GetOwner() );
}
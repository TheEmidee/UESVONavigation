#include "SVOPathfindingRenderingComponent.h"

#include "SVOPathFinderTest.h"

#include <Engine/Canvas.h>

FSVOPathFindingSceneProxy::FSVOPathFindingSceneProxy( const UPrimitiveComponent & component, const FSVOPathFindingSceneProxyData & proxy_data ) :
    FDebugRenderSceneProxy( &component )
{
    DrawType = SolidAndWireMeshes;
    TextWithoutShadowDistance = 1500;
    bWantsSelectionOutline = false;

    //Spheres = InSpheres;
    //Texts = InTexts;

    RenderingComponent = MakeWeakObjectPtr( const_cast< USVOPathFindingRenderingComponent * >( Cast< USVOPathFindingRenderingComponent >( &component ) ) );
    bDrawOnlyWhenSelected = RenderingComponent.IsValid() && RenderingComponent->DrawOnlyWhenSelected();

    Lines.Emplace( FDebugLine( RenderingComponent->GetPathFinderTest()->GetStartLocation(), RenderingComponent->GetPathFinderTest()->GetEndLocation(), FColor::Red, 5.0f ) );

    ActorOwner = component.GetOwner();
    /*QueryDataSource = Cast< const IEQSQueryResultSourceInterface >( ActorOwner );
    if ( QueryDataSource == nullptr )
    {
        QueryDataSource = Cast< const IEQSQueryResultSourceInterface >( &component );
    }*/

    //#if USE_EQS_DEBUGGER
    //    if ( Spheres.Num() == 0 && Texts.Num() == 0 && QueryDataSource != nullptr )
    //    {
    //        TArray< EQSDebug::FDebugHelper > DebugItems;
    //        FEQSSceneProxy::CollectEQSData( &component, QueryDataSource, Spheres, Texts, DebugItems );
    //    }
    //#endif
}

SIZE_T FSVOPathFindingSceneProxy::GetTypeHash() const
{
    static size_t UniquePointer;
    return reinterpret_cast< size_t >( &UniquePointer );
}

FPrimitiveViewRelevance FSVOPathFindingSceneProxy::GetViewRelevance( const FSceneView * view ) const
{
    FPrimitiveViewRelevance Result;
    Result.bDrawRelevance = /*view->Family->EngineShowFlags.GetSingleFlag(ViewFlagIndex) &&*/ IsShown( view ) && ( !bDrawOnlyWhenSelected || SafeIsActorSelected() );
    Result.bDynamicRelevance = true;
    // ideally the TranslucencyRelevance should be filled out by the material, here we do it conservative
    Result.bSeparateTranslucency = Result.bNormalTranslucency = IsShown( view );
    return Result;
}

bool FSVOPathFindingSceneProxy::SafeIsActorSelected() const
{
    if ( ActorOwner )
    {
        return ActorOwner->IsSelected();
    }

    return false;
}

USVOPathFindingRenderingComponent::USVOPathFindingRenderingComponent()
{
    DrawFlagName = "Navigation";
    bDrawOnlyWhenSelected = true;
}

FPrimitiveSceneProxy * USVOPathFindingRenderingComponent::CreateSceneProxy()
{
    FSVOPathFindingSceneProxyData proxy_data;
    GatherData( proxy_data, *GetPathFinderTest() );

    if ( FSVOPathFindingSceneProxy * new_scene_proxy = new FSVOPathFindingSceneProxy( *this, proxy_data ) )
    {
        if ( IsInGameThread() )
        {
            RenderingDebugDrawDelegateHelper.InitDelegateHelper( new_scene_proxy );
            RenderingDebugDrawDelegateHelper.ReregisterDebugDrawDelgate();
        }

        return new_scene_proxy;
    }

    return nullptr;
}

void USVOPathFindingRenderingComponent::CreateRenderState_Concurrent( FRegisterComponentContext * Context )
{
    Super::CreateRenderState_Concurrent( Context );

    RenderingDebugDrawDelegateHelper.RegisterDebugDrawDelgate();
}

void USVOPathFindingRenderingComponent::DestroyRenderState_Concurrent()
{
    RenderingDebugDrawDelegateHelper.UnregisterDebugDrawDelgate();

    Super::DestroyRenderState_Concurrent();
}

FBoxSphereBounds USVOPathFindingRenderingComponent::CalcBounds( const FTransform & LocalToWorld ) const
{
    FBoxSphereBounds result;

    if ( auto * owner = Cast< ASVOPathFinderTest >( GetOwner() ) )
    {
        FVector center, extent;
        owner->GetActorBounds( false, center, extent );
        result = FBoxSphereBounds( FBox::BuildAABB( center, extent ) );
    }

    return result;
}

void USVOPathFindingRenderingComponent::GatherData( FSVOPathFindingSceneProxyData & proxy_data, const ASVOPathFinderTest & path_finder_test )
{
}

void FSVOPathFindingRenderingDebugDrawDelegateHelper::DrawDebugLabels( UCanvas * Canvas, APlayerController * PC )
{
    if ( !ActorOwner || ( ActorOwner->IsSelected() == false && bDrawOnlyWhenSelected == true )
        // || (QueryDataSource && QueryDataSource->GetShouldDebugDrawLabels() == false)
    )
    {
        return;
    }

    // little hacky test but it's the only way to remove text rendering from bad worlds, when using UDebugDrawService for it
    if ( Canvas && Canvas->SceneView && Canvas->SceneView->Family && Canvas->SceneView->Family->Scene && Canvas->SceneView->Family->Scene->GetWorld() != ActorOwner->GetWorld() )
    {
        return;
    }

    FDebugDrawDelegateHelper::DrawDebugLabels( Canvas, PC );
}

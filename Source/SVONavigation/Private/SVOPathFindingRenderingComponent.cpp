#include "SVOPathfindingRenderingComponent.h"

#include "SVOPathFinderTest.h"

#include <Engine/Canvas.h>

void FSVOPathFindingSceneProxyData::GatherData( const ASVOPathFinderTest & path_finder_test )
{
    StartLocation = path_finder_test.GetStartLocation();
    EndLocation = path_finder_test.GetEndLocation();
    DebugInfos = path_finder_test.GetPathFinderDebugInfos();

    if ( path_finder_test.GetStepperLastStatus() == ESVOPathFindingAlgorithmStepperStatus::IsStopped )
    {
        PathFindingResult = TOptional< EGraphAStarResult >( path_finder_test.GetPathFindingResult() );
    }
    else
    {
        PathFindingResult.Reset();
    }
}

FSVOPathFindingSceneProxy::FSVOPathFindingSceneProxy( const UPrimitiveComponent & component, const FSVOPathFindingSceneProxyData & proxy_data ) :
    FDebugRenderSceneProxy( &component )
{
    DrawType = SolidAndWireMeshes;
    TextWithoutShadowDistance = 1500;
    bWantsSelectionOutline = false;
    ViewFlagName = TEXT( "Navigation" );
    ViewFlagIndex = static_cast< uint32 >( FEngineShowFlags::FindIndexByName( *ViewFlagName ) );

    RenderingComponent = MakeWeakObjectPtr( const_cast< USVOPathFindingRenderingComponent * >( Cast< USVOPathFindingRenderingComponent >( &component ) ) );
    PathFinderTest = RenderingComponent->GetPathFinderTest();
    DebugDrawOptions = PathFinderTest->GetDebugDrawOptions();
    
    const auto add_text = [ texts = &Texts ]( const FVector & start, const FVector & end, const FString & text ) {
        texts->Emplace( FText3d( text, FVector( 0.0f, 0.0f, 50.0f ) + ( start + end ) / 2.0f, FLinearColor::White ) );
    };

    Lines.Emplace( FDebugLine( proxy_data.DebugInfos.LastLastProcessedSingleNode.From.Location, proxy_data.DebugInfos.LastLastProcessedSingleNode.To.Location, FColor::Blue, 2.0f ) );

    for ( const auto & neighbor : proxy_data.DebugInfos.ProcessedNeighbors )
    {
        Lines.Emplace( FDebugLine( neighbor.From.Location, neighbor.To.Location, neighbor.bIsClosed ? FColor::Orange : FColor::Green, 1.0f ) );
    }

    /*const auto & debug_steps = proxy_data.DebugInfos.DebugSteps;
    FVector previous_location = proxy_data.StartLocation;

    auto iteration = 0;
    
    for ( const auto & debug_step : debug_steps )
    {
        Lines.Emplace( FDebugLine( previous_location, debug_step.CurrentLocationCost.Location, FColor::Blue, 4.0f ) );

        if ( DebugDrawOptions.bDrawCurrentCost )
        {
            add_text( previous_location, debug_step.CurrentLocationCost.Location, FString::Printf( TEXT( "Cost : %f" ), debug_step.CurrentLocationCost.Cost ) );   
        }

        previous_location = debug_step.CurrentLocationCost.Location;

        for ( const auto & neighbor_cost : debug_step.NeighborLocationCosts )
        {
            Lines.Emplace( FDebugLine( debug_step.CurrentLocationCost.Location, neighbor_cost.Location, neighbor_cost.WasEvaluated ? FColor::Green : FColor::Orange, 2.0f ) );

            if ( DebugDrawOptions.bDrawNeighborsCost && ( !DebugDrawOptions.bDrawOnlyLastNeighborsCost || iteration == debug_steps.Num() - 1 ) )
            {
                add_text( debug_step.CurrentLocationCost.Location, neighbor_cost.Location, FString::Printf( TEXT( "NeighborCost - %f" ), neighbor_cost.Cost ) );
            }
        }

        iteration++;
    }

    if ( DebugDrawOptions.bDrawBestPath )
    {
        const auto & best_path_points = proxy_data.DebugInfos.CurrentBestPath.GetPathPoints();

        for ( auto index = 0; index < best_path_points.Num() - 1; index++ )
        {
            Lines.Emplace( FDebugLine( best_path_points[ index ], best_path_points[ index + 1 ], FColor::Red, 6.0f ) );
        }        
    }*/

    if ( proxy_data.PathFindingResult.Get( EGraphAStarResult::SearchFail ) == EGraphAStarResult::SearchSuccess )
    {
        const auto & best_path_points = proxy_data.DebugInfos.CurrentBestPath.GetPathPoints();

        for ( auto index = 0; index < best_path_points.Num() - 1; index++ )
        {
            const auto from = best_path_points[ index ];
            const auto to = best_path_points[ index + 1 ];

            Lines.Emplace( FDebugLine( from, to, FColor::Red, 6.0f ) );
        }  
    }

    ActorOwner = component.GetOwner();
}

SIZE_T FSVOPathFindingSceneProxy::GetTypeHash() const
{
    static size_t UniquePointer;
    return reinterpret_cast< size_t >( &UniquePointer );
}

FPrimitiveViewRelevance FSVOPathFindingSceneProxy::GetViewRelevance( const FSceneView * view ) const
{
    FPrimitiveViewRelevance Result;
    Result.bDrawRelevance = /*view->Family->EngineShowFlags.GetSingleFlag(ViewFlagIndex) &&*/ IsShown( view ) && ( !DebugDrawOptions.bDrawOnlyWhenSelected || SafeIsActorSelected() );
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
}

FPrimitiveSceneProxy * USVOPathFindingRenderingComponent::CreateSceneProxy()
{
    FSVOPathFindingSceneProxyData proxy_data;
    GatherData( proxy_data, *GetPathFinderTest() );

    if ( FSVOPathFindingSceneProxy * new_scene_proxy = new FSVOPathFindingSceneProxy( *this, proxy_data ) )
    {
        if ( IsInGameThread() )
        {
            RenderingDebugDrawDelegateHelper.InitDelegateHelper( *new_scene_proxy );
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
    proxy_data.GatherData( path_finder_test );
}

void FSVOPathFindingRenderingDebugDrawDelegateHelper::InitDelegateHelper( const FSVOPathFindingSceneProxy & InSceneProxy )
{
    Super::InitDelegateHelper( &InSceneProxy );

    ActorOwner = InSceneProxy.ActorOwner;
    //QueryDataSource = InSceneProxy->QueryDataSource;
    DebugDrawOptions = InSceneProxy.DebugDrawOptions;
}

void FSVOPathFindingRenderingDebugDrawDelegateHelper::DrawDebugLabels( UCanvas * Canvas, APlayerController * PC )
{
    if ( !ActorOwner || ( ActorOwner->IsSelected() == false && DebugDrawOptions.bDrawOnlyWhenSelected == true ) )
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

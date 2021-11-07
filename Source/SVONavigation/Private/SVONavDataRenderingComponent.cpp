#include "SVONavDataRenderingComponent.h"

#include "SVONavigationData.h"

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
#include "Engine/Canvas.h"
#endif

#include "Debug/DebugDrawService.h"

#include <Engine/CollisionProfile.h>

#if WITH_EDITOR
#include "Editor.h"
#include "EditorViewportClient.h"
#endif

FSVONavigationMeshSceneProxy::FSVONavigationMeshSceneProxy( const UPrimitiveComponent * component ) :
    FDebugRenderSceneProxy( component )
{
    DrawType = EDrawType::SolidAndWireMeshes;

    RenderingComponent = MakeWeakObjectPtr( const_cast< USVONavDataRenderingComponent * >( Cast< USVONavDataRenderingComponent >( component ) ) );
    auto * navigation_data = Cast< ASVONavigationData >( RenderingComponent->GetOwner() );

    if ( navigation_data == nullptr )
    {
        return;
    }

    const auto & debug_infos = navigation_data->GetDebugInfos();
    const auto & svo_data = navigation_data->GetSVOData();
    const auto & all_navigation_bounds_data = svo_data.GetNavigationBoundsData();

    for ( const auto & navigation_bounds_data : all_navigation_bounds_data )
    {
        const auto & octree_data = navigation_bounds_data.GetOctreeData();
        const auto layer_count = octree_data.GetLayerCount();

        if ( layer_count == 0 )
        {
            continue;
        }

        const auto corrected_layer_index = FMath::Clamp( static_cast< int >( debug_infos.LayerIndexToDraw ), 0, layer_count - 1 );

        if ( debug_infos.bDebugDrawsBounds )
        {
            Boxes.Emplace( navigation_bounds_data.GetOctreeData().GetNavigationBounds(), FColor::White );
        }

        if ( debug_infos.bDebugDrawsLayers )
        {
            const auto half_voxel_size = navigation_bounds_data.GetOctreeData().GetLayer( corrected_layer_index ).GetVoxelHalfExtent();

            for ( const auto & node : octree_data.GetLayer( corrected_layer_index ).GetNodes() )
            {
                const auto code = node.MortonCode;
                const auto node_position = navigation_bounds_data.GetNodePosition( corrected_layer_index, code );
                const auto color = node.HasChildren() ? FColor::Blue : FColor::Green;
                Boxes.Emplace( FBox::BuildAABB( node_position, FVector( half_voxel_size ) ), color );
            }
        }

        /*if ( debug_infos.bDebugDrawsFreeLeaves )
        {
            for ( const auto & leaf : octree_data.GetLeaves() )
            {
                navigation_bounds_data.GetLinkPosition( leaf )
                Boxes.Emplace( leaf.GetBox(), FColor::Magenta );
            }
        }
        */

        const auto & leaves = octree_data.GetLeaves().GetLeaves();

        if ( debug_infos.bDebugDrawsFreeLeaves )
        {
            const auto leaf_subnode_half_extent = octree_data.GetLeaves().GetLeafSubNodeHalfExtent();

            for ( uint_fast32_t leaf_index = 0; leaf_index < static_cast< uint_fast32_t >( leaves.Num() ); leaf_index++ )
            {
                for ( uint8 leaf_voxel = 0; leaf_voxel < 64; leaf_voxel++ )
                {
                    if ( !leaves[ leaf_index ].IsSubNodeOccluded( leaf_voxel ) )
                    {
                        const FSVOOctreeLink link( 0, leaf_index, leaf_voxel );
                        const auto node_position = navigation_bounds_data.GetLinkPosition( link );

                        Boxes.Emplace( FBox::BuildAABB( node_position, FVector( leaf_subnode_half_extent ) ), FColor::Green );
                    }
                }
            }
        }
        
        if ( debug_infos.bDebugDrawsOccludedLeaves )
        {
            const auto leaf_subnode_half_extent = octree_data.GetLeaves().GetLeafSubNodeHalfExtent();

            for ( uint_fast32_t leaf_index = 0; leaf_index < static_cast< uint_fast32_t >( leaves.Num() ); leaf_index++ )
            {
                for ( uint8 leaf_voxel = 0; leaf_voxel < 64; leaf_voxel++ )
                {
                    if ( leaves[ leaf_index ].IsSubNodeOccluded( leaf_voxel ) )
                    {
                        const FSVOOctreeLink link( 0, leaf_index, leaf_voxel );
                        const auto node_position = navigation_bounds_data.GetLinkPosition( link );

                        Boxes.Emplace( FBox::BuildAABB( node_position, FVector( leaf_subnode_half_extent ) ), FColor::Orange );
                    }
                }
            }
        }
    }
}

FSVONavigationMeshSceneProxy::~FSVONavigationMeshSceneProxy()
{
}

SIZE_T FSVONavigationMeshSceneProxy::GetTypeHash() const
{
    static size_t UniquePointer;
    return reinterpret_cast< size_t >( &UniquePointer );
}

void FSVONavigationMeshSceneProxy::GetDynamicMeshElements( const TArray< const FSceneView * > & views, const FSceneViewFamily & view_family, uint32 visibility_map, FMeshElementCollector & collector ) const
{
    FDebugRenderSceneProxy::GetDynamicMeshElements( views, view_family, visibility_map, collector );

    for ( int32 view_index = 0; view_index < views.Num(); view_index++ )
    {
        if ( visibility_map & ( 1 << view_index ) )
        {
            const FSceneView * view = views[ view_index ];
            const bool bVisible = !!view->Family->EngineShowFlags.Navigation;

            if ( !bVisible )
            {
                continue;
            }

            /*FPrimitiveDrawInterface * pdi = collector.GetPDI( view_index );
            const auto line_thickness = ProxyData.GetDebugInfos().DebugLineThickness;

            const auto draw_boxes = [ pdi, line_thickness ]( const TArray< FBoxCenterAndExtent > & boxes, const FColor & color ) {
                for ( const auto & box : boxes )
                {
                    DrawDebugBox( pdi, box.Center, box.Extent, color, line_thickness );
                }
            };*/

            //draw_boxes( ProxyData.GetOctreeBounds(), FColor::White );
            //draw_boxes( ProxyData.GetLayers(), FColor::Blue );
            //draw_boxes( ProxyData.GetLeaves(), FColor::Magenta );
            //draw_boxes( ProxyData.GetOccludedLeaves(), FColor::Yellow );

            /*for ( const auto & line : ProxyData.GetLinks() )
            {
                pdi->DrawLine( line.Start, line.End, line.Color, SDPG_World, line.Thickness, 0, true );
            }*/
        }
    }
}

FPrimitiveViewRelevance FSVONavigationMeshSceneProxy::GetViewRelevance( const FSceneView * view ) const
{
    const bool bVisible = !!view->Family->EngineShowFlags.Navigation;
    FPrimitiveViewRelevance Result;
    Result.bDrawRelevance = bVisible && IsShown( view );
    Result.bDynamicRelevance = true;
    // ideally the TranslucencyRelevance should be filled out by the material, here we do it conservative
    Result.bSeparateTranslucency = Result.bNormalTranslucency = bVisible && IsShown( view );
    return Result;
}

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST

void FSVODebugDrawDelegateHelper::RegisterDebugDrawDelgate()
{
    ensureMsgf( State != RegisteredState, TEXT( "RegisterDebugDrawDelgate is already Registered!" ) );
    if ( State == InitializedState )
    {
        DebugTextDrawingDelegate = FDebugDrawDelegate::CreateRaw( this, &FSVODebugDrawDelegateHelper::DrawDebugLabels );
        DebugTextDrawingDelegateHandle = UDebugDrawService::Register( TEXT( "Navigation" ), DebugTextDrawingDelegate );
        State = RegisteredState;
    }
}

void FSVODebugDrawDelegateHelper::UnregisterDebugDrawDelgate()
{
    ensureMsgf( State != InitializedState, TEXT( "UnegisterDebugDrawDelgate is in an invalid State: %i !" ), State );
    if ( State == RegisteredState )
    {
        check( DebugTextDrawingDelegate.IsBound() );
        UDebugDrawService::Unregister( DebugTextDrawingDelegateHandle );
        State = InitializedState;
    }
}

void FSVODebugDrawDelegateHelper::DrawDebugLabels( UCanvas * Canvas, APlayerController * )
{
    if ( Canvas == nullptr )
    {
        return;
    }

    const bool bVisible = ( Canvas->SceneView && !!Canvas->SceneView->Family->EngineShowFlags.Navigation ); // || bForceRendering;
    if ( !bVisible /*|| bNeedsNewData*/ || DebugLabels.Num() == 0 )
    {
        return;
    }

    const FColor OldDrawColor = Canvas->DrawColor;
    Canvas->SetDrawColor( FColor::White );
    const FSceneView * View = Canvas->SceneView;
    UFont * Font = GEngine->GetSmallFont();
    const FDebugText * DebugText = DebugLabels.GetData();
    for ( int32 Idx = 0; Idx < DebugLabels.Num(); ++Idx, ++DebugText )
    {
        if ( View->ViewFrustum.IntersectSphere( DebugText->Location, 1.0f ) )
        {
            const FVector ScreenLoc = Canvas->Project( DebugText->Location );
            Canvas->DrawText( Font, DebugText->Text, ScreenLoc.X, ScreenLoc.Y );
        }
    }

    Canvas->SetDrawColor( OldDrawColor );
}
#endif

USVONavDataRenderingComponent::USVONavDataRenderingComponent()
{
    SetCollisionProfileName( UCollisionProfile::NoCollision_ProfileName );

    bIsEditorOnly = true;
    bSelectable = false;
    bForcesUpdate = false;
}

FPrimitiveSceneProxy * USVONavDataRenderingComponent::CreateSceneProxy()
{
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
    FSVONavigationMeshSceneProxy * proxy = nullptr;

    if ( IsVisible() )
    {
        if ( const ASVONavigationData * navigation_data = Cast< ASVONavigationData >( GetOwner() ) )
        {
            if ( navigation_data->IsDrawingEnabled() )
            {
                proxy = new FSVONavigationMeshSceneProxy( this );
            }
        }
    }

    if ( proxy != nullptr )
    {
        DebugDrawDelegateManager.InitDelegateHelper( proxy );
        DebugDrawDelegateManager.ReregisterDebugDrawDelgate();
    }

    return proxy;
#else
    return nullptr;
#endif
}

FBoxSphereBounds USVONavDataRenderingComponent::CalcBounds( const FTransform & LocalToWorld ) const
{
    FBox bounding_box( ForceInit );

    if ( ASVONavigationData * navigation_data = Cast< ASVONavigationData >( GetOwner() ) )
    {
        bounding_box = navigation_data->GetSVOData().GetBoundingBox();
    }

    return FBoxSphereBounds( bounding_box );
}

void USVONavDataRenderingComponent::CreateRenderState_Concurrent( FRegisterComponentContext * context )
{
    Super::CreateRenderState_Concurrent( context );

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
    DebugDrawDelegateManager.RegisterDebugDrawDelgate();
#endif
}

void USVONavDataRenderingComponent::DestroyRenderState_Concurrent()
{
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
    DebugDrawDelegateManager.UnregisterDebugDrawDelgate();
#endif

    Super::DestroyRenderState_Concurrent();
}

bool USVONavDataRenderingComponent::IsNavigationShowFlagSet( const UWorld * world )
{
    bool show_navigation = false;

    FWorldContext * world_context = GEngine->GetWorldContextFromWorld( world );

#if WITH_EDITOR
    if ( GEditor != nullptr && world_context && world_context->WorldType != EWorldType::Game )
    {
        show_navigation = world_context->GameViewport != nullptr && world_context->GameViewport->EngineShowFlags.Navigation;
        if ( show_navigation == false )
        {
            // we have to check all viewports because we can't to distinguish between SIE and PIE at this point.
            for ( FEditorViewportClient * current_viewport : GEditor->GetAllViewportClients() )
            {
                if ( current_viewport && current_viewport->EngineShowFlags.Navigation )
                {
                    show_navigation = true;
                    break;
                }
            }
        }
    }
    else
#endif //WITH_EDITOR
    {
        show_navigation = world_context && world_context->GameViewport && world_context->GameViewport->EngineShowFlags.Navigation;
    }

    return show_navigation;
}
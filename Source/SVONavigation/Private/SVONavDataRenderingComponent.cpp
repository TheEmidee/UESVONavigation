#include "SVONavDataRenderingComponent.h"

#include "SVOHelpers.h"
#include "SVONavigationData.h"

#include <Debug/DebugDrawService.h>
#include <Engine/CollisionProfile.h>

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
#include <Engine/Canvas.h>
#endif

#if WITH_EDITOR
#include <Editor.h>
#include <EditorViewportClient.h>
#endif

static const FColor OccludedVoxelColor = FColor::Orange;
static const FColor FreeVoxelColor = FColor::Green;

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
    //const auto & svo_data = navigation_data->GetSVOData();
    const auto & all_navigation_bounds_data = navigation_data->GetVolumeNavigationData();

    for ( const auto & navigation_bounds_data : all_navigation_bounds_data )
    {
        const auto & octree_data = navigation_bounds_data.GetData();
        const auto layer_count = octree_data.GetLayerCount();

        if ( layer_count == 0 )
        {
            continue;
        }

        if ( debug_infos.bDebugDrawBounds )
        {
            Boxes.Emplace( navigation_bounds_data.GetData().GetNavigationBounds(), FColor::White );
        }

        const auto try_add_node_text_infos = [ this, &debug_infos, &navigation_bounds_data ]( const MortonCode node_morton_code, const LayerIndex node_layer_index, const FVector & node_position ) {
            if ( debug_infos.bDebugDrawMortonCoords )
            {
                const FIntVector morton_coords = FIntVector( FSVOHelpers::GetVectorFromMortonCode( node_morton_code ) );

                const auto vertical_offset_ratio = navigation_bounds_data.GetLayerRatio( node_layer_index ) * 20.0f;

                MortonTexts.Emplace( FString::Printf( TEXT( "%i:%llu" ), node_layer_index, node_morton_code ), node_position/*, FLinearColor::White*/ );
                MortonTexts.Emplace( FString::Printf( TEXT( "%s" ), *morton_coords.ToString() ), node_position + FVector( 0.0f, 0.0f, 10.0f * vertical_offset_ratio )/*, FLinearColor::Black*/ );
                MortonTexts.Emplace( FString::Printf( TEXT( "%s" ), *node_position.ToCompactString() ), node_position + FVector( 0.0f, 0.0f, 20.0f * vertical_offset_ratio )/*, FLinearColor::Red*/ );
            }
        };

        const auto try_add_voxel_to_boxes = [ this, &debug_infos ]( const FVector & voxel_location, const float voxel_half_extent, const bool is_occluded ) {
            if ( debug_infos.DebugDrawFreeVoxels && !is_occluded )
            {
                Boxes.Emplace( FBox::BuildAABB( voxel_location, FVector( voxel_half_extent ) ), FreeVoxelColor );

                return true;
            }
            if ( debug_infos.DebugDrawOccludedVoxels && is_occluded )
            {
                Boxes.Emplace( FBox::BuildAABB( voxel_location, FVector( voxel_half_extent ) ), OccludedVoxelColor );

                return true;
            }

            return false;
        };

        if ( debug_infos.bDebugDrawLayers )
        {
            const auto corrected_layer_index = FMath::Clamp( static_cast< int >( debug_infos.LayerIndexToDraw ), 0, layer_count - 1 );
            const auto half_voxel_size = navigation_bounds_data.GetData().GetLayer( corrected_layer_index ).GetVoxelHalfExtent();

            for ( const auto & node : octree_data.GetLayer( corrected_layer_index ).GetNodes() )
            {
                const auto code = node.MortonCode;
                const FSVONodeAddress node_address( corrected_layer_index, code );

                const auto node_position = navigation_bounds_data.GetNodePositionFromAddress( node_address );

                if ( try_add_voxel_to_boxes( node_position, half_voxel_size, node.HasChildren() ) )
                {
                    try_add_node_text_infos( code, corrected_layer_index, node_position );
                }
            }
        }

        if ( debug_infos.bDebugDrawSubNodes )
        {
            const auto & leaf_layer = octree_data.GetLayer( 0 );
            const auto leaf_subnode_half_extent = octree_data.GetLeaves().GetLeafSubNodeHalfExtent();
            const auto leaf_subnode_extent = octree_data.GetLeaves().GetLeafSubNodeExtent();
            const auto leaf_voxel_half_extent = leaf_layer.GetVoxelHalfExtent();

            for ( const auto & leaf_node : leaf_layer.GetNodes() )
            {
                const auto leaf_position = navigation_bounds_data.GetNodePositionFromAddress( FSVONodeAddress( 0, leaf_node.MortonCode ) );

                if ( leaf_node.HasChildren() )
                {
                    const auto & leaf = octree_data.GetLeaves().GetLeaf( leaf_node.FirstChild.NodeIndex );

                    for ( SubNodeIndex subnode_index = 0; subnode_index < 64; subnode_index++ )
                    {
                        const auto morton_coords = FSVOHelpers::GetVectorFromMortonCode( subnode_index );
                        const auto subnode_location = leaf_position - leaf_voxel_half_extent + morton_coords * leaf_subnode_extent + leaf_subnode_half_extent;
                        const bool is_subnode_occluded = leaf.IsSubNodeOccluded( subnode_index );

                        if ( try_add_voxel_to_boxes( subnode_location, leaf_subnode_half_extent, is_subnode_occluded ) )
                        {
                            try_add_node_text_infos( subnode_index, 0, subnode_location );
                        }
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

void FSVODebugDrawDelegateHelper::DrawDebugLabels( UCanvas * Canvas, APlayerController * pc )
{
    /*if ( Canvas == nullptr )
    {
        return;
    }

    FDebugDrawDelegateHelper::DrawDebugLabels( Canvas, pc );
    return;*/

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
        bounding_box = navigation_data->GetBoundingBox();
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
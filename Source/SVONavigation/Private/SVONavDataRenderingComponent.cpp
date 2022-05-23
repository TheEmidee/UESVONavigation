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
    NavigationData = MakeWeakObjectPtr( Cast< ASVONavigationData >( RenderingComponent->GetOwner() ) );

    if ( NavigationData == nullptr )
    {
        return;
    }

    const auto & debug_infos = NavigationData->GetDebugInfos();
    const auto & all_navigation_bounds_data = NavigationData->GetVolumeNavigationData();

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
            auto vertical_offset = 0.0f;
            if ( debug_infos.bDebugDrawMortonCoords )
            {
                Texts.Emplace( FString::Printf( TEXT( "%i:%llu" ), node_layer_index, node_morton_code ), node_position, FLinearColor::Black );
                vertical_offset += 40.0f;
            }
            if ( debug_infos.bDebugDrawNodeAddress )
            {
                const FIntVector morton_coords = FIntVector( FSVOHelpers::GetVectorFromMortonCode( node_morton_code ) );
                Texts.Emplace( FString::Printf( TEXT( "%s" ), *morton_coords.ToString() ), node_position + FVector( 0.0f, 0.0f, vertical_offset ), FLinearColor::Black );
                vertical_offset += 40.0f;
            }
            if ( debug_infos.bDebugDrawNodeLocation )
            {
                Texts.Emplace( FString::Printf( TEXT( "%s" ), *node_position.ToCompactString() ), node_position + FVector( 0.0f, 0.0f, vertical_offset ), FLinearColor::Black );
            }
        };

        const auto try_add_voxel_to_boxes = [ this, &debug_infos ]( const FVector & voxel_location, const float node_extent, const bool is_occluded ) {
            if ( debug_infos.bDebugDrawFreeVoxels && !is_occluded )
            {
                Boxes.Emplace( FBox::BuildAABB( voxel_location, FVector( node_extent ) ), FreeVoxelColor );

                return true;
            }
            if ( debug_infos.bDebugDrawOccludedVoxels && is_occluded )
            {
                Boxes.Emplace( FBox::BuildAABB( voxel_location, FVector( node_extent ) ), OccludedVoxelColor );

                return true;
            }

            return false;
        };

        const auto & leaf_nodes = octree_data.GetLeafNodes();
        const auto leaf_node_extent = leaf_nodes.GetLeafNodeExtent();

        if ( debug_infos.bDebugDrawLayers )
        {
            const auto corrected_layer_index = FMath::Clamp( static_cast< int >( debug_infos.LayerIndexToDraw ), 0, layer_count - 1 );
            const auto node_extent = navigation_bounds_data.GetData().GetLayer( corrected_layer_index ).GetNodeExtent();

            for ( const auto & node : octree_data.GetLayer( corrected_layer_index ).GetNodes() )
            {
                const auto code = node.MortonCode;

                if ( corrected_layer_index == 0 )
                {
                    const auto leaf_node_position = navigation_bounds_data.GetLeafNodePositionFromMortonCode( code );

                    if ( try_add_voxel_to_boxes( leaf_node_position, leaf_node_extent, node.HasChildren() ) )
                    {
                        try_add_node_text_infos( code, 0, leaf_node_position );
                    }
                }
                else
                {
                    const FSVONodeAddress node_address( corrected_layer_index, code );
                    const auto node_position = navigation_bounds_data.GetNodePositionFromAddress( node_address, false );

                    if ( try_add_voxel_to_boxes( node_position, node_extent, node.HasChildren() ) )
                    {
                        try_add_node_text_infos( code, corrected_layer_index, node_position );
                    }
                }
            }
        }

        if ( debug_infos.bDebugDrawSubNodes )
        {
            const auto & leaf_layer = octree_data.GetLayer( 0 );
            const auto leaf_sub_node_size = leaf_nodes.GetLeafSubNodeSize();
            const auto leaf_sub_node_extent = leaf_nodes.GetLeafSubNodeExtent();

            for ( const auto & leaf_node : leaf_layer.GetNodes() )
            {
                if ( leaf_node.HasChildren() )
                {
                    const auto & leaf = octree_data.GetLeafNodes().GetLeafNode( leaf_node.FirstChild.NodeIndex );
                    const auto code = leaf_node.MortonCode;
                    const auto leaf_node_position = navigation_bounds_data.GetLeafNodePositionFromMortonCode( code );

                    for ( SubNodeIndex sub_node_index = 0; sub_node_index < 64; sub_node_index++ )
                    {
                        const auto sub_node_morton_coords = FSVOHelpers::GetVectorFromMortonCode( sub_node_index );
                        const auto sub_node_location = leaf_node_position - leaf_node_extent + sub_node_morton_coords * leaf_sub_node_size + leaf_sub_node_extent;
                        const bool is_sub_node_occluded = leaf.IsSubNodeOccluded( sub_node_index );

                        if ( try_add_voxel_to_boxes( sub_node_location, leaf_sub_node_extent, is_sub_node_occluded ) )
                        {
                            try_add_node_text_infos( sub_node_index, 0, sub_node_location );
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

void FSVODebugDrawDelegateHelper::InitDelegateHelper( const FSVONavigationMeshSceneProxy * scene_proxy )
{
    Super::InitDelegateHelper( scene_proxy );

    NavigationData = scene_proxy->NavigationData;
}

void FSVODebugDrawDelegateHelper::RegisterDebugDrawDelegateInternal()
{
    ensureMsgf( State != RegisteredState, TEXT( "RegisterDebugDrawDelgate is already Registered!" ) );
    if ( State == InitializedState )
    {
        DebugTextDrawingDelegate = FDebugDrawDelegate::CreateRaw( this, &FSVODebugDrawDelegateHelper::DrawDebugLabels );
        DebugTextDrawingDelegateHandle = UDebugDrawService::Register( TEXT( "Navigation" ), DebugTextDrawingDelegate );
        State = RegisteredState;
    }
}

void FSVODebugDrawDelegateHelper::UnregisterDebugDrawDelegate()
{
    ensureMsgf( State != InitializedState, TEXT( "UnegisterDebugDrawDelgate is in an invalid State: %i !" ), State );
    if ( State == RegisteredState )
    {
        check( DebugTextDrawingDelegate.IsBound() );
        UDebugDrawService::Unregister( DebugTextDrawingDelegateHandle );
        State = InitializedState;
    }
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
        DebugDrawDelegateManager.ReregisterDebugDrawDelegate();
    }

    return proxy;
#else
    return nullptr;
#endif
}

FBoxSphereBounds USVONavDataRenderingComponent::CalcBounds( const FTransform & LocalToWorld ) const
{
    FBox bounding_box( ForceInit );

    if ( const ASVONavigationData * navigation_data = Cast< ASVONavigationData >( GetOwner() ) )
    {
        bounding_box = navigation_data->GetBoundingBox();
    }

    return FBoxSphereBounds( bounding_box );
}

void USVONavDataRenderingComponent::CreateRenderState_Concurrent( FRegisterComponentContext * context )
{
    Super::CreateRenderState_Concurrent( context );

#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
    DebugDrawDelegateManager.RequestRegisterDebugDrawDelegate( context );
#endif
}

void USVONavDataRenderingComponent::DestroyRenderState_Concurrent()
{
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
    DebugDrawDelegateManager.UnregisterDebugDrawDelegate();
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
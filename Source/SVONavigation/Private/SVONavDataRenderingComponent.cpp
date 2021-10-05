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

namespace
{
    void DrawDebugBox( FPrimitiveDrawInterface * pdi, FVector const & center, FVector const & box, FColor const & color, const float line_thickness )
    {
        // no debug line drawing on dedicated server
        if ( pdi == nullptr )
        {
            return;
        }

        pdi->DrawLine( center + FVector( box.X, box.Y, box.Z ), center + FVector( box.X, -box.Y, box.Z ), color, SDPG_World, line_thickness );
        pdi->DrawLine( center + FVector( box.X, -box.Y, box.Z ), center + FVector( -box.X, -box.Y, box.Z ), color, SDPG_World, line_thickness );
        pdi->DrawLine( center + FVector( -box.X, -box.Y, box.Z ), center + FVector( -box.X, box.Y, box.Z ), color, SDPG_World, line_thickness );
        pdi->DrawLine( center + FVector( -box.X, box.Y, box.Z ), center + FVector( box.X, box.Y, box.Z ), color, SDPG_World, line_thickness );

        pdi->DrawLine( center + FVector( box.X, box.Y, -box.Z ), center + FVector( box.X, -box.Y, -box.Z ), color, SDPG_World, line_thickness );
        pdi->DrawLine( center + FVector( box.X, -box.Y, -box.Z ), center + FVector( -box.X, -box.Y, -box.Z ), color, SDPG_World, line_thickness );
        pdi->DrawLine( center + FVector( -box.X, -box.Y, -box.Z ), center + FVector( -box.X, box.Y, -box.Z ), color, SDPG_World, line_thickness );
        pdi->DrawLine( center + FVector( -box.X, box.Y, -box.Z ), center + FVector( box.X, box.Y, -box.Z ), color, SDPG_World, line_thickness );

        pdi->DrawLine( center + FVector( box.X, box.Y, box.Z ), center + FVector( box.X, box.Y, -box.Z ), color, SDPG_World, line_thickness );
        pdi->DrawLine( center + FVector( box.X, -box.Y, box.Z ), center + FVector( box.X, -box.Y, -box.Z ), color, SDPG_World, line_thickness );
        pdi->DrawLine( center + FVector( -box.X, -box.Y, box.Z ), center + FVector( -box.X, -box.Y, -box.Z ), color, SDPG_World, line_thickness );
        pdi->DrawLine( center + FVector( -box.X, box.Y, box.Z ), center + FVector( -box.X, box.Y, -box.Z ), color, SDPG_World, line_thickness );
    }
}

void FSVONavigationSceneProxyData::Reset()
{
    OctreeBounds.Reset();
}

void FSVONavigationSceneProxyData::Serialize( FArchive & archive )
{
    auto octree_bounds_count = OctreeBounds.Num();
    archive << octree_bounds_count;
    if ( archive.IsLoading() )
    {
        OctreeBounds.Reset( octree_bounds_count );
        OctreeBounds.AddUninitialized( octree_bounds_count );
    }

    for ( auto index = 0; index < octree_bounds_count; index++ )
    {
        archive << OctreeBounds[ index ].Center;
        archive << OctreeBounds[ index ].Extent;
    }

    auto links_count = Links.Num();
    archive << links_count;
    if ( archive.IsLoading() )
    {
        Links.Reset( links_count );
        Links.AddUninitialized( links_count );
    }

    for ( auto index = 0; index < links_count; index++ )
    {
        archive << Links[ index ].Thickness;
        archive << Links[ index ].Start;
        archive << Links[ index ].End;
        archive << Links[ index ].Color;
    }

    int32 num_texts = DebugTexts.Num();
    archive << num_texts;
    if ( archive.IsLoading() )
    {
        DebugTexts.SetNum( num_texts );
    }

    for ( int32 index = 0; index < num_texts; index++ )
    {
        archive << DebugTexts[ index ].Location;
        archive << DebugTexts[ index ].Text;
    }
}

uint32 FSVONavigationSceneProxyData::GetAllocatedSize() const
{
    return OctreeBounds.GetAllocatedSize() +
           Layers.GetAllocatedSize() +
           Leaves.GetAllocatedSize() +
           OccludedLeaves.GetAllocatedSize() +
           Links.GetAllocatedSize() +
           DebugTexts.GetAllocatedSize();
}

void FSVONavigationSceneProxyData::GatherData( const ASVONavigationData & navigation_data )
{
    Reset();

    DebugInfos = navigation_data.GetDebugInfos();

    const auto & svo_data = navigation_data.GetSVOData();
    const auto & navigation_bounds = svo_data.GetNavigationBoundsData();

    if ( DebugInfos.ItDebugDrawsBounds )
    {
        for ( const auto & bounds_data : navigation_bounds )
        {
            OctreeBounds.Emplace( FBoxCenterAndExtent( bounds_data.GetNavigationBounds() ) );
        }
    }

    const auto it_draws_layers = DebugInfos.ItDebugDrawsLayers;
    const auto it_draws_leaves = DebugInfos.ItDebugDrawsLeaves;
    const auto it_draws_morton_codes = DebugInfos.ItDebugDrawsMortonCodes;

    if ( it_draws_layers || it_draws_leaves || it_draws_morton_codes )
    {
        for ( const auto & bounds_data : navigation_bounds )
        {
            const auto & octree_data = bounds_data.GetOctreeData();

            if ( octree_data.NodesByLayers.Num() == 0 )
            {
                break;
            }

            const auto fill_array = [ &octree_data, &bounds_data ]( TArray< FBoxCenterAndExtent > & array, const LayerIndex layer_index ) {
                const auto & layer_nodes = octree_data.NodesByLayers[ layer_index ];
                for ( const auto & node : layer_nodes )
                {
                    const auto code = node.MortonCode;
                    const auto node_position = bounds_data.GetNodePosition( layer_index, code );
                    const auto half_voxel_size = bounds_data.GetLayerVoxelHalfSize( layer_index );

                    array.Emplace( FBoxCenterAndExtent( node_position, FVector( half_voxel_size ) ) );
                }
            };

            if ( it_draws_leaves )
            {
                fill_array( Leaves, 0 );
            }

            if ( it_draws_layers )
            {
                const auto layer_index_to_draw = DebugInfos.LayerIndexToDraw;

                if ( layer_index_to_draw >= 1 && layer_index_to_draw < octree_data.NodesByLayers.Num() )
                {
                    fill_array( Layers, layer_index_to_draw );
                }
            }

            if ( it_draws_morton_codes )
            {
                const auto morton_codes_layer_index = DebugInfos.MortonCodeLayerIndexToDraw;

                if ( morton_codes_layer_index >= 0 && morton_codes_layer_index < octree_data.NodesByLayers.Num() )
                {
                    const auto & layer_nodes = octree_data.NodesByLayers[ morton_codes_layer_index ];

                    for ( const auto & node : layer_nodes )
                    {
                        DebugTexts.Emplace( FDebugText( bounds_data.GetNodePosition( morton_codes_layer_index, node.MortonCode ),
                            FString::Printf( TEXT( "%i:%llu" ), morton_codes_layer_index, node.MortonCode ) ) );
                    }
                }
            }
        }
    }

    if ( DebugInfos.ItDebugDrawsOccludedLeaves )
    {
        for ( const auto & bounds_data : navigation_bounds )
        {
            const auto occluded_leaf_voxel_size = bounds_data.GetLayerVoxelHalfSize( 0 ) * 0.25f;
            const auto & octree_data = bounds_data.GetOctreeData();

            for ( uint_fast32_t I = 0; I < static_cast< uint_fast32_t >( octree_data.Leaves.Num() ); I++ )
            {
                for ( uint8 J = 0; J < 64; J++ )
                {
                    if ( octree_data.Leaves[ I ].GetSubNode( J ) )
                    {
                        const FSVOOctreeLink link { 0, I, J };
                        const auto node_location = bounds_data.GetNodePositionFromLink( link );

                        OccludedLeaves.Emplace( FBoxCenterAndExtent( node_location, FVector( occluded_leaf_voxel_size ) ) );
                    }
                }
            }
        }
    }

    if ( DebugInfos.ItDebugDrawsLinks )
    {
        for ( const auto & bounds_data : navigation_bounds )
        {
            const auto & octree_data = bounds_data.GetOctreeData();

            if ( octree_data.NodesByLayers.Num() == 0 )
            {
                break;
            }

            const auto layer_index_to_draw = DebugInfos.LinksLayerIndexToDraw;

            if ( layer_index_to_draw >= 1 && layer_index_to_draw < octree_data.NodesByLayers.Num() )
            {
                const auto & layer_nodes = octree_data.NodesByLayers[ layer_index_to_draw ];
                for ( const auto & node : layer_nodes )
                {
                    const auto code = node.MortonCode;
                    const auto node_position = bounds_data.GetNodePosition( layer_index_to_draw, code );

                    const auto add_link = [ &bounds_data, &node_position, line_thickness = DebugInfos.DebugLineThickness ]( TArray< FDebugRenderSceneProxy::FDebugLine > & links, const FSVOOctreeLink & link ) {
                        if ( !link.IsValid() )
                        {
                            return;
                        }

                        const auto neighbor_position = bounds_data.GetNodePositionFromLink( link );
                        links.Emplace( FDebugRenderSceneProxy::FDebugLine( node_position, neighbor_position, FColor::Orange, line_thickness ) );
                    };

                    if ( DebugInfos.ItDebugDrawsNeighborLinks )
                    {
                        for ( const auto & link : node.Neighbors )
                        {
                            add_link( Links, link );
                        }
                    }

                    if ( DebugInfos.ItDebugDrawsParentLinks )
                    {
                        const auto & link = node.Parent;

                        add_link( Links, link );
                    }

                    if ( DebugInfos.ItDebugDrawsFirstChildLinks )
                    {
                        const auto & link = node.FirstChild;

                        add_link( Links, link );
                    }
                }
            }
        }
    }
}

SIZE_T FSVONavigationMeshSceneProxy::GetTypeHash() const
{
    static size_t UniquePointer;
    return reinterpret_cast< size_t >( &UniquePointer );
}

FSVONavigationMeshSceneProxy::FSVONavigationMeshSceneProxy( const UPrimitiveComponent * component, FSVONavigationSceneProxyData * proxy_data ) :
    FDebugRenderSceneProxy( component )
{
    DrawType = EDrawType::SolidAndWireMeshes;

    if ( proxy_data )
    {
        ProxyData = *proxy_data;
    }

    RenderingComponent = MakeWeakObjectPtr( const_cast< USVONavDataRenderingComponent * >( Cast< USVONavDataRenderingComponent >( component ) ) );
}

FSVONavigationMeshSceneProxy::~FSVONavigationMeshSceneProxy()
{
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

            FPrimitiveDrawInterface * pdi = collector.GetPDI( view_index );
            const auto line_thickness = ProxyData.GetDebugInfos().DebugLineThickness;

            const auto draw_boxes = [ pdi, line_thickness ]( const TArray< FBoxCenterAndExtent > & boxes, const FColor & color ) {
                for ( const auto & box : boxes )
                {
                    DrawDebugBox( pdi, box.Center, box.Extent, color, line_thickness );
                }
            };

            draw_boxes( ProxyData.GetOctreeBounds(), FColor::White );
            draw_boxes( ProxyData.GetLayers(), FColor::Blue );
            draw_boxes( ProxyData.GetLeaves(), FColor::Magenta );
            draw_boxes( ProxyData.GetOccludedLeaves(), FColor::Yellow );

            for ( const auto & line : ProxyData.GetLinks() )
            {
                pdi->DrawLine( line.Start, line.End, line.Color, SDPG_World, line.Thickness, 0, true );
            }
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
                FSVONavigationSceneProxyData ProxyData;
                GatherData( ProxyData, *navigation_data );

                proxy = new FSVONavigationMeshSceneProxy( this, &ProxyData );
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

void USVONavDataRenderingComponent::GatherData( FSVONavigationSceneProxyData & proxy_data, const ASVONavigationData & navigation_data ) const
{
    proxy_data.GatherData( navigation_data );
}

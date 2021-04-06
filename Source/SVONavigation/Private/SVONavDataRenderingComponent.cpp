#include "SVONavDataRenderingComponent.h"

#include "SVONavigationData.h"

#include <Engine/CollisionProfile.h>

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
}

uint32 FSVONavigationSceneProxyData::GetAllocatedSize() const
{
    return OctreeBounds.GetAllocatedSize() +
           Layers.GetAllocatedSize() +
           Leaves.GetAllocatedSize() +
           OccludedLeaves.GetAllocatedSize() +
           Links.GetAllocatedSize();
}

void FSVONavigationSceneProxyData::GatherData( const ASVONavigationData & navigation_data )
{
    Reset();

    DebugInfos = navigation_data.GetDebugInfos();

    const auto & navigation_bounds = navigation_data.GetNavigationBoundsData();

    if ( DebugInfos.ItDebugDrawsBounds )
    {
        for ( const auto & bounds_data : navigation_bounds )
        {
            OctreeBounds.Emplace( FBoxCenterAndExtent( bounds_data.Value.GetBox() ) );
        }
    }

    const auto it_draws_layers = DebugInfos.ItDebugDrawsLayers;
    const auto it_draws_leaves = DebugInfos.ItDebugDrawsLeaves;

    if ( it_draws_layers || it_draws_leaves )
    {
        for ( const auto & bounds_data : navigation_bounds )
        {
            const auto & octree_data = bounds_data.Value.GetOctreeData();

            if ( octree_data.NodesByLayers.Num() == 0 )
            {
                break;
            }

            const auto fill_array = [ &octree_data, &bounds_data ]( TArray< FBoxCenterAndExtent > & array, const LayerIndex layer_index ) {
                const auto & layer_nodes = octree_data.NodesByLayers[ layer_index ];
                for ( const auto & node : layer_nodes )
                {
                    const auto code = node.MortonCode;
                    const auto node_position = bounds_data.Value.GetNodePosition( layer_index, code );
                    const auto half_voxel_size = bounds_data.Value.GetLayerVoxelHalfSize( layer_index );

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
        }
    }

    if ( DebugInfos.ItDebugDrawsOccludedLeaves )
    {
        for ( const auto & bounds_data : navigation_bounds )
        {
            const auto occluded_leaf_voxel_size = bounds_data.Value.GetLayerVoxelHalfSize( 0 ) * 0.25f;
            const auto & octree_data = bounds_data.Value.GetOctreeData();

            for ( uint_fast32_t I = 0; I < static_cast< uint_fast32_t >( octree_data.Leaves.Num() ); I++ )
            {
                for ( uint8 J = 0; J < 64; J++ )
                {
                    if ( octree_data.Leaves[ I ].GetSubNode( J ) )
                    {
                        const FSVOOctreeLink link { 0, I, J };
                        const auto node_location = bounds_data.Value.GetNodePositionFromLink( link );

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
            const auto & octree_data = bounds_data.Value.GetOctreeData();

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
                    const auto node_position = bounds_data.Value.GetNodePosition( layer_index_to_draw, code );

                    const auto add_link = [ &bounds_data, &node_position, line_thickness = DebugInfos.DebugLineThickness ]( TArray< FDebugRenderSceneProxy::FDebugLine > & links, const FSVOOctreeLink & link )
                    {
                        if ( !link.IsValid() )
                        {
                            return;
                        }

                        const auto neighbor_position = bounds_data.Value.GetNodePositionFromLink( link );
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

USVONavDataRenderingComponent::USVONavDataRenderingComponent()
{
    SetCollisionProfileName( UCollisionProfile::NoCollision_ProfileName );

    bIsEditorOnly = true;
    bSelectable = false;
}

FPrimitiveSceneProxy * USVONavDataRenderingComponent::CreateSceneProxy()
{
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
    FSVONavigationMeshSceneProxy * proxy = nullptr;

    if ( IsVisible() )
    {
        if ( const ASVONavigationData * navigation_data = Cast< ASVONavigationData >( GetOwner() ) )
        {
            if ( navigation_data->GetDebugInfos().ItHasDebugDrawingEnabled )
            {
                FSVONavigationSceneProxyData ProxyData;
                GatherData( ProxyData, *navigation_data );

                proxy = new FSVONavigationMeshSceneProxy( this, &ProxyData );
            }
        }
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
        const auto & navigation_bounds = navigation_data->GetNavigationBoundsData();

        for ( const auto & bounds : navigation_bounds )
        {
            bounding_box += bounds.Value.GetBox();
        }
    }

    return FBoxSphereBounds( bounding_box );
}

void USVONavDataRenderingComponent::GatherData( FSVONavigationSceneProxyData & proxy_data, const ASVONavigationData & navigation_data ) const
{
    proxy_data.GatherData( navigation_data );
}

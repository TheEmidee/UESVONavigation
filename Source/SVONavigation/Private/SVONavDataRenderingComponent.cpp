#include "SVONavDataRenderingComponent.h"

#include "SVONavigationData.h"

#include <Engine/CollisionProfile.h>

namespace
{
    void DrawDebugBox( FPrimitiveDrawInterface * PDI, FVector const & Center, FVector const & Box, FColor const & Color )
    {
        // no debug line drawing on dedicated server
        if ( PDI != nullptr )
        {
            PDI->DrawLine( Center + FVector( Box.X, Box.Y, Box.Z ), Center + FVector( Box.X, -Box.Y, Box.Z ), Color, SDPG_World, 5.0f );
            PDI->DrawLine( Center + FVector( Box.X, -Box.Y, Box.Z ), Center + FVector( -Box.X, -Box.Y, Box.Z ), Color, SDPG_World, 5.0f );
            PDI->DrawLine( Center + FVector( -Box.X, -Box.Y, Box.Z ), Center + FVector( -Box.X, Box.Y, Box.Z ), Color, SDPG_World, 5.0f );
            PDI->DrawLine( Center + FVector( -Box.X, Box.Y, Box.Z ), Center + FVector( Box.X, Box.Y, Box.Z ), Color, SDPG_World, 5.0f );

            PDI->DrawLine( Center + FVector( Box.X, Box.Y, -Box.Z ), Center + FVector( Box.X, -Box.Y, -Box.Z ), Color, SDPG_World, 5.0f );
            PDI->DrawLine( Center + FVector( Box.X, -Box.Y, -Box.Z ), Center + FVector( -Box.X, -Box.Y, -Box.Z ), Color, SDPG_World, 5.0f );
            PDI->DrawLine( Center + FVector( -Box.X, -Box.Y, -Box.Z ), Center + FVector( -Box.X, Box.Y, -Box.Z ), Color, SDPG_World, 5.0f );
            PDI->DrawLine( Center + FVector( -Box.X, Box.Y, -Box.Z ), Center + FVector( Box.X, Box.Y, -Box.Z ), Color, SDPG_World, 5.0f );

            PDI->DrawLine( Center + FVector( Box.X, Box.Y, Box.Z ), Center + FVector( Box.X, Box.Y, -Box.Z ), Color, SDPG_World, 5.0f );
            PDI->DrawLine( Center + FVector( Box.X, -Box.Y, Box.Z ), Center + FVector( Box.X, -Box.Y, -Box.Z ), Color, SDPG_World, 5.0f );
            PDI->DrawLine( Center + FVector( -Box.X, -Box.Y, Box.Z ), Center + FVector( -Box.X, -Box.Y, -Box.Z ), Color, SDPG_World, 5.0f );
            PDI->DrawLine( Center + FVector( -Box.X, Box.Y, Box.Z ), Center + FVector( -Box.X, Box.Y, -Box.Z ), Color, SDPG_World, 5.0f );
        }
    }

    bool HasDebugDrawFlag( int32 flags, ESVODebugDrawFlags test_flag )
    {
        return ( flags & ( 1 << static_cast< int32 >( test_flag ) ) ) != 0;
    }
}

void FSVONavigationSceneProxyData::Reset()
{
    OctreeBounds.Reset();
}

void FSVONavigationSceneProxyData::Serialize( FArchive & Ar )
{
    int32 NumBounds = OctreeBounds.Num();
    Ar << NumBounds;
    if ( Ar.IsLoading() )
    {
        OctreeBounds.SetNum( NumBounds );
    }

    for ( int32 Idx = 0; Idx < NumBounds; Idx++ )
    {
        Ar << OctreeBounds[ Idx ].Center;
        Ar << OctreeBounds[ Idx ].Extent;
    }
}

uint32 FSVONavigationSceneProxyData::GetAllocatedSize() const
{
    return OctreeBounds.GetAllocatedSize() +
           Layers.GetAllocatedSize() +
           Leaves.GetAllocatedSize() +
           OccludedLeaves.GetAllocatedSize();
}

void FSVONavigationSceneProxyData::GatherData( const ASVONavigationData * navigation_data, int32 debug_draw_flags )
{
    Reset();

    DebugDrawFlags = debug_draw_flags;

    const auto & navigation_bounds = navigation_data->GetNavigationBoundsData();

    if ( HasDebugDrawFlag( DebugDrawFlags, ESVODebugDrawFlags::Bounds ) )
    {
        for ( const auto & bounds_data : navigation_bounds )
        {
            OctreeBounds.Emplace( FBoxCenterAndExtent( bounds_data.Value.GetBox() ) );
        }
    }

    const auto it_draws_layers = HasDebugDrawFlag( DebugDrawFlags, ESVODebugDrawFlags::Layers );
    const auto it_draws_leaves = HasDebugDrawFlag( DebugDrawFlags, ESVODebugDrawFlags::Leaves );

    if ( it_draws_layers || it_draws_leaves )
    {
        for ( const auto & bounds_data : navigation_bounds )
        {
            const auto & octree_data = bounds_data.Value.GetOctreeData();

            if ( octree_data.NodesByLayers.Num() == 0 )
            {
                break;
            }

            const auto fill_array = [ &octree_data, &bounds_data ]( TArray< FBoxCenterAndExtent > & array, const LayerIndex layer_index )
            {
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
                const auto layer_index_to_draw = navigation_data->GetLayerIndexToDraw();

                if ( layer_index_to_draw >= 1 && layer_index_to_draw < octree_data.NodesByLayers.Num() )
                {
                    fill_array( Layers, layer_index_to_draw );
                }
            }
        }
    }

    if ( HasDebugDrawFlag( DebugDrawFlags, ESVODebugDrawFlags::OccludesLeaves ) )
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

            const auto draw_boxes = [ pdi ]( const TArray< FBoxCenterAndExtent > & boxes, const FColor & color )
            {
                for ( const auto & box : boxes )
                {
                    DrawDebugBox( pdi, box.Center, box.Extent, color );
                }
            };

            draw_boxes( ProxyData.GetOctreeBounds(), FColor::White );
            draw_boxes( ProxyData.GetLayers(), FColor::Blue );
            draw_boxes( ProxyData.GetLeaves(), FColor::Magenta );
            draw_boxes( ProxyData.GetOccludedLeaves(), FColor::Yellow );
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
            if ( navigation_data->HasDebugDrawingEnabled() )
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
    proxy_data.GatherData( &navigation_data, navigation_data.GetDebugDrawFlags() );
}

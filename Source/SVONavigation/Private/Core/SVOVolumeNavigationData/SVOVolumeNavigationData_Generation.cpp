#include "SVOVolumeNavigationData.h"

#include "SVOHelpers.h"
#include "SVONavigationData.h"

#include "Engine/OverlapResult.h"

FSVOVolumeNavigationDataGenerationSettings::FSVOVolumeNavigationDataGenerationSettings() :
    VoxelExtent( 0.0f ),
    World( nullptr )
{
}

void FSVOVolumeNavigationData::GenerateNavigationData( const FBox & volume_bounds, const FSVOVolumeNavigationDataGenerationSettings & generation_settings )
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GenerateNavigationData );

    Settings = generation_settings;
    VolumeBounds = volume_bounds;

    const auto voxel_extent = Settings.VoxelExtent;

    if ( !SVOData.Initialize( voxel_extent, VolumeBounds ) )
    {
        return;
    }

    const auto layer_count = SVOData.GetLayerCount();

    FirstPassRasterization();

    {
        QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_AllocateLeafNodes );
        const auto leaf_count = SVOData.GetLayerBlockedNodes( 0 ).Num() * 8;
        SVOData.GetLeafNodes().AllocateLeafNodes( leaf_count );
    }

    TMap< LeafIndex, MortonCode > leaf_index_to_parent_morton_code;
    RasterizeInitialLayer( leaf_index_to_parent_morton_code );

    for ( LayerIndex layer_index = 1; layer_index < layer_count; ++layer_index )
    {
        RasterizeLayer( layer_index );
    }

    BuildParentLinkForLeafNodes( leaf_index_to_parent_morton_code );

    for ( LayerIndex layer_index = layer_count - 2; layer_index != static_cast< LayerIndex >( -1 ); --layer_index )
    {
        BuildNeighborLinks( layer_index );
    }

    SVOData.bIsValid = true;
}

bool FSVOVolumeNavigationData::IsPositionOccluded( const FVector & position, const float box_extent ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_IsPositionOccluded );
    TArray< FOverlapResult > overlap_results;
    const auto result = Settings.World->OverlapMultiByChannel(  
        overlap_results,
        position,
        FQuat::Identity,
        Settings.GenerationSettings.CollisionChannel,
        FCollisionShape::MakeBox( FVector( box_extent + Settings.GenerationSettings.Clearance ) ),
        Settings.GenerationSettings.CollisionQueryParameters );

    if ( !result )
    {
        return false;
    }

    return overlap_results.FindByPredicate( []( const FOverlapResult & overlap_result ) {
        return overlap_result.GetComponent()->CanEverAffectNavigation();
    } ) != nullptr;
}

void FSVOVolumeNavigationData::FirstPassRasterization()
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_FirstPassRasterization );
    {
        const auto & layer = SVOData.GetLayer( 1 );
        const auto layer_max_node_count = layer.GetMaxNodeCount();
        const auto layer_node_extent = layer.GetNodeExtent();        

        for ( MortonCode node_index = 0; node_index < layer_max_node_count; ++node_index )
        {
            const auto position = GetNodePositionFromLayerAndMortonCode( 1, node_index );
            
            if ( IsPositionOccluded( position, layer_node_extent ) )
            {
                SVOData.AddBlockedNode( 0, node_index );
            }
        }
    }

    {
        for ( int32 layer_index = 1; layer_index < GetLayerCount(); layer_index++ )
        {
            const auto & parent_layer_blocked_nodes = SVOData.GetLayerBlockedNodes( layer_index - 1 );
            for ( const MortonCode morton_code : parent_layer_blocked_nodes )
            {
                SVOData.AddBlockedNode( layer_index, FSVOHelpers::GetParentMortonCode( morton_code ) );
            }
        }
    }
}

void FSVOVolumeNavigationData::RasterizeLeaf( const FVector & node_position, const LeafIndex leaf_index )
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_RasterizeLeaf );

    const auto leaf_node_extent = SVOData.GetLeafNodes().GetLeafNodeExtent();
    const auto leaf_sub_node_size = SVOData.GetLeafNodes().GetLeafSubNodeSize();
    const auto leaf_sub_node_extent = SVOData.GetLeafNodes().GetLeafSubNodeExtent();
    const auto location = node_position - leaf_node_extent;

    for ( SubNodeIndex sub_node_index = 0; sub_node_index < 64; sub_node_index++ )
    {
        const auto morton_coords = FSVOHelpers::GetVectorFromMortonCode( sub_node_index );
        const auto leaf_node_location = location + morton_coords * leaf_sub_node_size + leaf_sub_node_extent;
        const bool is_leaf_occluded = IsPositionOccluded( leaf_node_location, leaf_sub_node_extent );

        SVOData.GetLeafNodes().AddLeafNode( leaf_index, sub_node_index, is_leaf_occluded );
    }
}

void FSVOVolumeNavigationData::RasterizeInitialLayer( TMap< LeafIndex, MortonCode > & leaf_index_to_layer_one_node_index_map )
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_RasterizeInitialLayer );

    auto & layer_zero = SVOData.GetLayer( 0 );
    auto & layer_zero_nodes = layer_zero.GetNodes();

    LeafIndex leaf_index = 0;
    const auto & layer_zero_blocked_nodes = SVOData.GetLayerBlockedNodes( 0 );
    const auto layer_one_blocked_node_count = layer_zero_blocked_nodes.Num();
    layer_zero_nodes.Reserve( layer_one_blocked_node_count * 8 );

    const auto layer_max_node_count = layer_zero.GetMaxNodeCount();

    auto & leaf_nodes = SVOData.GetLeafNodes();
    const auto leaf_node_extent = leaf_nodes.GetLeafNodeExtent();

    for ( NodeIndex node_index = 0; node_index < layer_max_node_count; node_index++ )
    {
        const auto parent_morton_code = FSVOHelpers::GetParentMortonCode( node_index );
        const auto is_blocked = layer_zero_blocked_nodes.Contains( parent_morton_code );

        if ( !is_blocked )
        {
            continue;
        }

        auto & layer_zero_node = layer_zero_nodes.Emplace_GetRef();
        layer_zero_node.MortonCode = node_index;

        const auto leaf_node_position = GetLeafNodePositionFromMortonCode( layer_zero_node.MortonCode );

        leaf_index_to_layer_one_node_index_map.Add( leaf_index, parent_morton_code );

        if ( IsPositionOccluded( leaf_node_position, leaf_node_extent ) )
        {
            RasterizeLeaf( leaf_node_position, leaf_index );
            layer_zero_node.FirstChild.LayerIndex = 0;
            layer_zero_node.FirstChild.NodeIndex = leaf_index;
            layer_zero_node.FirstChild.SubNodeIndex = 0;
        }
        else
        {
            leaf_nodes.AddEmptyLeafNode();
            layer_zero_node.FirstChild.Invalidate();
        }

        leaf_index++;
    }
}

void FSVOVolumeNavigationData::RasterizeLayer( const LayerIndex layer_index )
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_RasterizeLayer );

    auto & layer = SVOData.GetLayer( layer_index );
    auto & layer_nodes = layer.GetNodes();
    const auto & layer_blocked_nodes = SVOData.GetLayerBlockedNodes( layer_index );

    checkf( layer_index > 0 && layer_index < GetLayerCount(), TEXT( "layer_index is out of bounds" ) );

    layer_nodes.Reserve( layer_blocked_nodes.Num() * 8 );

    const auto layer_max_node_count = layer.GetMaxNodeCount();

    for ( NodeIndex node_index = 0; node_index < layer_max_node_count; node_index++ )
    {
        const auto is_blocked = layer_blocked_nodes.Contains( FSVOHelpers::GetParentMortonCode( node_index ) );

        if ( !is_blocked )
        {
            continue;
        }

        const auto new_node_index = layer_nodes.Emplace();

        auto & layer_node = layer_nodes[ new_node_index ];
        layer_node.MortonCode = node_index;

        const auto child_layer_index = layer_index - 1;
        const auto first_child_morton_code = FSVOHelpers::GetFirstChildMortonCode( layer_node.MortonCode );
        const auto child_index_from_code = GetNodeIndexFromMortonCode( child_layer_index, first_child_morton_code );

        auto & first_child = layer_node.FirstChild;

        if ( child_index_from_code != INDEX_NONE )
        {
            // Set parent->child links
            first_child.LayerIndex = child_layer_index;
            first_child.NodeIndex = child_index_from_code;

            auto & child_layer = SVOData.GetLayer( child_layer_index );

            // Set child->parent links
            for ( auto child_index = 0; child_index < 8; ++child_index )
            {
                auto & child_node = child_layer.GetNodes()[ first_child.NodeIndex + child_index ];

                child_node.Parent.LayerIndex = layer_index;
                child_node.Parent.NodeIndex = new_node_index;
            }
        }
        else
        {
            first_child.Invalidate();
        }
    }
}

int32 FSVOVolumeNavigationData::GetNodeIndexFromMortonCode( const LayerIndex layer_index, const MortonCode morton_code ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetNodeIndexFromMortonCode );

    const auto & layer_nodes = SVOData.GetLayer( layer_index ).GetNodes();

    // Since nodes are ordered, we can use the binary search
    return Algo::BinarySearch( layer_nodes, FSVONode( morton_code ) );
}
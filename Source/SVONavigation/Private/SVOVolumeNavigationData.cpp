#include "SVOVolumeNavigationData.h"

#include "SVOHelpers.h"
#include "SVONavigationData.h"
#include "SVONavigationTypes.h"
#include "SVOVersion.h"

#include <ThirdParty/libmorton/morton.h>

static constexpr FIntVector NeighborDirections[ 6 ] = {
    { 1, 0, 0 },
    { -1, 0, 0 },
    { 0, 1, 0 },
    { 0, -1, 0 },
    { 0, 0, 1 },
    { 0, 0, -1 }
};

FSVOVolumeNavigationDataGenerationSettings::FSVOVolumeNavigationDataGenerationSettings() :
    VoxelExtent( 0.0f ),
    World( nullptr )
{
}

FVector FSVOVolumeNavigationData::GetNodePositionFromAddress( const FSVONodeAddress & address ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetNodePositionFromNodeAddress );

    const auto & layer = SVOData.GetLayer( address.LayerIndex );

    const auto voxel_size = layer.GetVoxelExtent();
    const auto voxel_half_size = layer.GetVoxelHalfExtent();
    const auto morton_coords = FSVOHelpers::GetVectorFromMortonCode( address.NodeIndex );
    const auto & navigation_bounds = SVOData.GetNavigationBounds();

    const auto navigation_bounds_center = navigation_bounds.GetCenter();
    const auto navigation_bounds_extent = navigation_bounds.GetExtent();
    const auto position = navigation_bounds_center - navigation_bounds_extent + morton_coords * voxel_size + voxel_half_size;

    return position;
}

FVector FSVOVolumeNavigationData::GetSubNodePositionFromAddress( const FSVONodeAddress & address ) const
{
    checkf( address.LayerIndex == 0, TEXT( "To get the position of a node, you must use GetNodePositionFromAddress" ) )

        QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetSubNodePositionFromAddress );

    const auto node_position = GetNodePositionFromAddress( address );

    const auto & leaf_layer = SVOData.GetLayer( 0 );
    const auto leaf_voxel_half_extent = leaf_layer.GetVoxelHalfExtent();
    const auto & leaves = SVOData.GetLeaves();
    const auto leaf_subnode_extent = leaves.GetLeafSubNodeExtent();
    const auto leaf_subnode_half_extent = leaves.GetLeafSubNodeHalfExtent();

    const auto morton_coords = FSVOHelpers::GetVectorFromMortonCode( address.SubNodeIndex );
    return node_position - leaf_voxel_half_extent + morton_coords * leaf_subnode_extent + leaf_subnode_half_extent;
}

bool FSVOVolumeNavigationData::GetNodeAddressFromPosition( FSVONodeAddress & node_address, const FVector & position ) const
{
    const auto & navigation_bounds = SVOData.GetNavigationBounds();

    if ( !navigation_bounds.IsInside( position ) )
    {
        return false;
    }

    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetNodeAddressFromPosition );

    FVector origin;
    FVector extent;

    navigation_bounds.GetCenterAndExtents( origin, extent );
    // The z-order origin of the volume (where code == 0)
    const auto z_origin = origin - extent;
    // The local position of the point in volume space
    const auto local_position = position - z_origin;

    const auto layer_count = GetLayerCount();
    LayerIndex layer_index = layer_count - 1;
    NodeIndex nodeIndex = 0;

    while ( layer_index >= 0 && layer_index < layer_count )
    {
        const auto & layer = SVOData.GetLayer( layer_index );
        const auto & layer_nodes = layer.GetNodes();
        const auto voxel_size = layer.GetVoxelExtent();

        FIntVector voxel_coords;
        voxel_coords.X = FMath::FloorToInt( local_position.X / voxel_size );
        voxel_coords.Y = FMath::FloorToInt( local_position.Y / voxel_size );
        voxel_coords.Z = FMath::FloorToInt( local_position.Z / voxel_size );

        // Get the morton code we want for this layer
        const auto code = FSVOHelpers::GetMortonCodeFromVector( voxel_coords );
        const auto half_voxel_size = layer.GetVoxelHalfExtent();

        for ( NodeIndex node_index = nodeIndex; node_index < static_cast< uint32 >( layer_nodes.Num() ); node_index++ )
        {
            const auto & node = layer_nodes[ node_index ];

            // This is the node we are in
            if ( node.MortonCode != code )
            {
                continue;
            }

            // There are no child nodes, so this is our nav position
            if ( !node.FirstChild.IsValid() ) // && layerIndex > 0)
            {
                node_address.LayerIndex = layer_index;
                node_address.NodeIndex = node_index;
                node_address.SubNodeIndex = 0;
                return true;
            }

            // If this is a leaf node, we need to find our subnode
            if ( layer_index == 0 )
            {
                const auto & leaves = SVOData.GetLeaves();
                const auto & leaf = leaves.GetLeaf( node.FirstChild.NodeIndex );

                // We need to calculate the node local position to get the morton code for the leaf
                // The world position of the 0 node
                const auto node_position = GetNodePositionFromAddress( FSVONodeAddress( layer_index, node.MortonCode ) );
                // The morton origin of the node
                const auto node_origin = node_position - FVector( half_voxel_size );
                // The requested position, relative to the node origin
                const auto node_local_position = position - node_origin;
                // Now get our voxel coordinates
                const auto voxel_quarter_size = voxel_size * 0.25f;

                FIntVector leaf_coords;
                leaf_coords.X = FMath::FloorToInt( node_local_position.X / voxel_quarter_size );
                leaf_coords.Y = FMath::FloorToInt( node_local_position.Y / voxel_quarter_size );
                leaf_coords.Z = FMath::FloorToInt( node_local_position.Z / voxel_quarter_size );

                node_address.LayerIndex = 0;
                node_address.NodeIndex = node_index;

                const auto leaf_code = FSVOHelpers::GetMortonCodeFromVector( leaf_coords ); // This morton code is our key into the 64-bit leaf node

                if ( leaf.IsSubNodeOccluded( leaf_code ) )
                {
                    return false; // This voxel is blocked
                }

                node_address.SubNodeIndex = leaf_code;

                return true;
            }

            // If we've got here, the current node has a child, and isn't a leaf, so lets go down...
            layer_index = layer_nodes[ node_index ].FirstChild.LayerIndex;
            nodeIndex = layer_nodes[ node_index ].FirstChild.NodeIndex;

            break; //stop iterating this layer
        }
    }

    return false;
}

void FSVOVolumeNavigationData::GetNodeNeighbors( TArray< FSVONodeAddress > & neighbors, const FSVONodeAddress & node_address ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetNeighbors );

    const auto & node = GetNodeFromAddress( node_address );
    if ( node_address.LayerIndex == 0 && node.FirstChild.IsValid() )
    {
        GetLeafNeighbors( neighbors, node_address );
        return;
    }

    for ( NeighborDirection neighbor_direction = 0; neighbor_direction < 6; neighbor_direction++ )
    {
        const auto & neighbor_address = node.Neighbors[ neighbor_direction ];

        if ( !neighbor_address.IsValid() )
        {
            continue;
        }

        const auto & neighbor = GetNodeFromAddress( neighbor_address );

        if ( !neighbor.HasChildren() )
        {
            neighbors.Add( neighbor_address );
            continue;
        }

        TArray< FSVONodeAddress > neighbor_addresses_working_set;
        neighbor_addresses_working_set.Push( neighbor_address );

        while ( neighbor_addresses_working_set.Num() > 0 )
        {
            // Pop off the top of the working set
            auto this_address = neighbor_addresses_working_set.Pop();

            const auto & this_node = GetNodeFromAddress( this_address );

            // If the node as no children, it's clear, so add to neighbors and continue
            if ( !this_node.HasChildren() )
            {
                neighbors.Add( neighbor_address );
                continue;
            }

            if ( this_address.LayerIndex > 0 )
            {
                /* Morton code node ordering
                    Z
                    ^
                    |          5 --- 7
                    |        / |   / |
                    |       4 --- 6  |
                    |  X    |  1 -|- 3
                    | /     | /   | /
                    |/      0 --- 2
                    +-------------------> Y
                */

                static constexpr NodeIndex ChildOffsetsDirections[ 6 ][ 4 ] = {
                    { 0, 4, 2, 6 },
                    { 1, 3, 5, 7 },
                    { 0, 1, 4, 5 },
                    { 2, 3, 6, 7 },
                    { 0, 1, 2, 3 },
                    { 4, 5, 6, 7 }
                };

                // If it's above layer 0, we will need to potentially add 4 children using our offsets
                for ( const auto & child_index : ChildOffsetsDirections[ neighbor_direction ] )
                {
                    auto first_child_address = this_node.FirstChild;
                    first_child_address.NodeIndex += child_index;
                    const auto & child_node = GetNodeFromAddress( first_child_address );

                    if ( child_node.HasChildren() ) // If it has children, add them to the working set to keep going down
                    {
                        neighbor_addresses_working_set.Emplace( first_child_address );
                    }
                    else
                    {
                        neighbors.Emplace( first_child_address );
                    }
                }
            }
            else
            {
                /*
                Sub node morton code ordering for the face pointing to neighbor[0], which is (1,0,0)
                Use the debug draw options of the navigation data in the scene to show all the sub nodes
                 
                Z
                |
                |   36 38 52 54
                |   32 34 48 50
                |   04 06 20 22
                |   00 02 16 18
                |
                ------------------ Y
                */

                static constexpr NodeIndex LeafChildOffsetsDirections[ 6 ][ 16 ] = {
                    { 0, 2, 16, 18, 4, 6, 20, 22, 32, 34, 48, 50, 36, 38, 52, 54 },
                    { 9, 11, 25, 27, 13, 15, 29, 31, 41, 43, 57, 59, 45, 47, 61, 63 },
                    { 0, 1, 8, 9, 4, 5, 12, 13, 32, 33, 40, 41, 36, 37, 44, 45 },
                    { 18, 19, 26, 27, 22, 23, 30, 31, 50, 51, 58, 59, 54, 55, 62, 63 },
                    { 0, 1, 8, 9, 2, 3, 10, 11, 16, 17, 24, 25, 18, 19, 26, 27 },
                    { 36, 37, 44, 45, 38, 39, 46, 47, 52, 53, 60, 61, 54, 55, 62, 63 }
                };

                // If this is a leaf layer, then we need to add whichever of the 16 facing leaf nodes aren't blocked
                for ( const auto & leaf_index : LeafChildOffsetsDirections[ neighbor_direction ] )
                {
                    // Each of the childnodes
                    auto first_child_address = neighbor.FirstChild;
                    const auto & leaf_node = SVOData.GetLeaves().GetLeaf( first_child_address.NodeIndex );

                    first_child_address.SubNodeIndex = leaf_index;

                    if ( !leaf_node.IsSubNodeOccluded( leaf_index ) )
                    {
                        neighbors.Emplace( first_child_address );
                    }
                }
            }
        }
    }
}

float FSVOVolumeNavigationData::GetLayerRatio( const LayerIndex layer_index ) const
{
    return static_cast< float >( layer_index ) / GetLayerCount();
}

float FSVOVolumeNavigationData::GetLayerInverseRatio( const LayerIndex layer_index ) const
{
    return 1.0f - GetLayerRatio( layer_index );
}

float FSVOVolumeNavigationData::GetVoxelHalfExtentFromNodeAddress( const FSVONodeAddress node_address ) const
{
    if ( node_address.LayerIndex == 0 )
    {
        return SVOData.GetLeaves().GetLeafSubNodeHalfExtent();
    }

    return SVOData.GetLayer( node_address.LayerIndex ).GetVoxelHalfExtent();
}

TOptional< FNavLocation > FSVOVolumeNavigationData::GetRandomPoint() const
{
    TArray< FSVONodeAddress > non_occluded_nodes;
    const FSVONodeAddress top_most_node_address( GetLayerCount(), 0, 0 );

    GetFreeNodesFromNodeAddress( top_most_node_address, non_occluded_nodes );

    if ( non_occluded_nodes.Num() == 0 )
    {
        return TOptional< FNavLocation >();
    }

    const auto random_index = FMath::RandRange( 0, non_occluded_nodes.Num() - 1 );
    const auto random_node = non_occluded_nodes[ random_index ];
    const auto random_node_location = GetNodePositionFromAddress( random_node );
    const auto random_node_half_extent = GetVoxelHalfExtentFromNodeAddress( random_node );

    const auto node_bounds = FBox::BuildAABB( random_node_location, FVector( random_node_half_extent ) );
    const auto random_point_in_node = FMath::RandPointInBox( node_bounds );
    return FNavLocation( random_point_in_node, random_node.GetNavNodeRef() );
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
        const auto leaf_count = SVOData.GetLayer( 0 ).GetBlockedNodesCount() * 8;
        SVOData.GetLeaves().AllocateLeaves( leaf_count );
    }

    RasterizeInitialLayer();

    for ( LayerIndex layer_index = 1; layer_index < layer_count; ++layer_index )
    {
        RasterizeLayer( layer_index );
    }

    for ( LayerIndex layer_index = layer_count - 2; layer_index != static_cast< LayerIndex >( -1 ); --layer_index )
    {
        BuildNeighborLinks( layer_index );
    }

    SVOData.bIsValid = true;
}

void FSVOVolumeNavigationData::Serialize( FArchive & archive, const ESVOVersion /*version*/ )
{
    archive << VolumeBounds;
    archive << SVOData;
}

bool FSVOVolumeNavigationData::IsPositionOccluded( const FVector & position, const float box_half_extent ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_IsPositionOccluded );
    return Settings.World->OverlapBlockingTestByChannel(
        position,
        FQuat::Identity,
        Settings.GenerationSettings.CollisionChannel,
        FCollisionShape::MakeBox( FVector( box_half_extent + Settings.GenerationSettings.Clearance ) ),
        Settings.GenerationSettings.CollisionQueryParameters );
}

void FSVOVolumeNavigationData::FirstPassRasterization()
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_FirstPassRasterization );
    {
        const auto & layer = SVOData.GetLayer( 1 );
        const auto layer_max_node_count = layer.GetMaxNodeCount();
        const auto layer_voxel_half_extent = layer.GetVoxelHalfExtent();

        auto & layer_zero = SVOData.GetLayer( 0 );

        for ( MortonCode node_index = 0; node_index < layer_max_node_count; ++node_index )
        {
            const auto position = GetNodePositionFromAddress( FSVONodeAddress( 1, node_index ) );

            if ( IsPositionOccluded( position, layer_voxel_half_extent ) )
            {
                layer_zero.AddBlockedNode( node_index );
            }
        }
    }

    {
        for ( int32 layer_index = 1; layer_index < GetLayerCount(); layer_index++ )
        {
            auto & layer = SVOData.GetLayer( layer_index );
            auto & parent_layer = SVOData.GetLayer( layer_index - 1 );
            for ( MortonCode morton_code : parent_layer.GetBlockedNodes() )
            {
                layer.AddBlockedNode( FSVOHelpers::GetParentMortonCode( morton_code ) );
            }
        }
    }
}

void FSVOVolumeNavigationData::RasterizeLeaf( const FVector & node_position, const LeafIndex leaf_index )
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_RasterizeLeaf );

    const auto leaf_half_extent = SVOData.GetLeaves().GetLeafHalfExtent();
    const auto leaf_subnode_extent = SVOData.GetLeaves().GetLeafSubNodeExtent();
    const auto leaf_subnode_half_extent = SVOData.GetLeaves().GetLeafSubNodeHalfExtent();
    const auto location = node_position - leaf_half_extent;

    for ( SubNodeIndex subnode_index = 0; subnode_index < 64; subnode_index++ )
    {
        const auto morton_coords = FSVOHelpers::GetVectorFromMortonCode( subnode_index );
        const auto voxel_location = location + morton_coords * leaf_subnode_extent + leaf_subnode_half_extent;
        const bool is_leaf_occluded = IsPositionOccluded( voxel_location, leaf_subnode_half_extent );

        SVOData.GetLeaves().AddLeaf( leaf_index, subnode_index, is_leaf_occluded );
    }
}

void FSVOVolumeNavigationData::RasterizeInitialLayer()
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_RasterizeInitialLayer );

    auto & layer_zero = SVOData.GetLayer( 0 );
    auto & layer_zero_nodes = layer_zero.GetNodes();

    LeafIndex leaf_index = 0;

    const auto layer_one_blocked_node_count = SVOData.GetLayer( 1 ).GetBlockedNodesCount();
    layer_zero_nodes.Reserve( layer_one_blocked_node_count * 8 );

    const auto layer_max_node_count = layer_zero.GetMaxNodeCount();
    const auto layer_voxel_half_size = layer_zero.GetVoxelHalfExtent();

    for ( NodeIndex node_index = 0; node_index < layer_max_node_count; node_index++ )
    {
        const auto parent_morton_code = FSVOHelpers::GetParentMortonCode( node_index );
        const auto is_blocked = layer_zero.GetBlockedNodes().Contains( parent_morton_code );

        // If we know this node needs to be added, from the low res first pass
        if ( !is_blocked )
        {
            continue;
        }

        auto & layer_zero_node = layer_zero_nodes.Emplace_GetRef();
        layer_zero_node.MortonCode = node_index;

        const auto node_position = GetNodePositionFromAddress( FSVONodeAddress( 0, node_index ) );

        // Now check if we have any blocking, and search leaf nodes
        if ( IsPositionOccluded( node_position, layer_voxel_half_size ) )
        {
            RasterizeLeaf( node_position, leaf_index );
            layer_zero_node.FirstChild.LayerIndex = 0;
            layer_zero_node.FirstChild.NodeIndex = leaf_index;
            layer_zero_node.FirstChild.SubNodeIndex = 0;
        }
        else
        {
            SVOData.GetLeaves().AddEmptyLeaf();
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
    const auto & layer_blocked_nodes = layer.GetBlockedNodes();

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

void FSVOVolumeNavigationData::BuildNeighborLinks( const LayerIndex layer_index )
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_BuildNeighborLinks );

    auto & layer_nodes = SVOData.GetLayer( layer_index ).GetNodes();
    const auto max_layer_index = GetLayerCount() - 2;

    for ( NodeIndex layer_node_index = 0; layer_node_index < static_cast< uint32 >( layer_nodes.Num() ); layer_node_index++ )
    {
        auto & node = layer_nodes[ layer_node_index ];
        FVector node_position = GetNodePositionFromAddress( FSVONodeAddress( layer_index, node.MortonCode ) );

        for ( NeighborDirection direction = 0; direction < 6; direction++ )
        {
            NodeIndex node_index = layer_node_index;
            FSVONodeAddress & neighbor_address = node.Neighbors[ direction ];
            LayerIndex current_layer = layer_index;

            while ( !FindNeighborInDirection( neighbor_address, current_layer, node_index, direction, node_position ) && current_layer < max_layer_index )
            {
                auto & parent_address = SVOData.GetLayer( current_layer ).GetNodes()[ node_index ].Parent;
                if ( parent_address.IsValid() )
                {
                    node_index = parent_address.NodeIndex;
                    current_layer = parent_address.LayerIndex;
                }
                else
                {
                    current_layer++;
                    const auto node_index_from_morton = GetNodeIndexFromMortonCode( current_layer, FSVOHelpers::GetParentMortonCode( node.MortonCode ) );
                    ensure( node_index_from_morton != INDEX_NONE );
                    node_index = static_cast< NodeIndex >( node_index_from_morton );
                }
            }
        }
    }
}

bool FSVOVolumeNavigationData::FindNeighborInDirection( FSVONodeAddress & node_address, const LayerIndex layer_index, const NodeIndex node_index, const NeighborDirection direction, const FVector & node_position )
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_FindNeighborInDirection );

    const auto max_coordinates = static_cast< int32 >( SVOData.GetLayer( layer_index ).GetMaxNodeCount() );
    const auto & layer_nodes = SVOData.GetLayer( layer_index ).GetNodes();
    const auto layer_nodes_count = layer_nodes.Num();
    const auto & target_node = layer_nodes[ node_index ];

    FIntVector neighbor_coords( FSVOHelpers::GetVectorFromMortonCode( target_node.MortonCode ) );
    neighbor_coords += NeighborDirections[ direction ];

    if ( neighbor_coords.X < 0 || neighbor_coords.X >= max_coordinates ||
         neighbor_coords.Y < 0 || neighbor_coords.Y >= max_coordinates ||
         neighbor_coords.Z < 0 || neighbor_coords.Z >= max_coordinates )
    {
        node_address.Invalidate();
        return true;
    }

    const auto neighbor_code = FSVOHelpers::GetMortonCodeFromVector( neighbor_coords );

    int32 stop_index = layer_nodes_count;
    int32 increment = 1;

    if ( neighbor_code < target_node.MortonCode )
    {
        increment = -1;
        stop_index = -1;
    }

    for ( int32 neighbor_node_index = node_index + increment; neighbor_node_index != stop_index; neighbor_node_index += increment )
    {
        auto & node = layer_nodes[ neighbor_node_index ];

        if ( node.MortonCode == neighbor_code )
        {
            if ( layer_index == 0 &&
                 node.HasChildren() &&
                 SVOData.GetLeaves().GetLeaf( node.FirstChild.NodeIndex ).IsCompletelyOccluded() )
            {
                node_address.Invalidate();
                return true;
            }

            node_address.LayerIndex = layer_index;

            if ( neighbor_node_index >= layer_nodes_count || neighbor_node_index < 0 )
            {
                break;
            }

            node_address.NodeIndex = neighbor_node_index;

            return true;
        }

        // If we've passed the code we're looking for, it's not on this layer
        if ( increment == -1 && node.MortonCode < neighbor_code || increment == 1 && node.MortonCode > neighbor_code )
        {
            return false;
        }
    }
    return false;
}

void FSVOVolumeNavigationData::GetLeafNeighbors( TArray< FSVONodeAddress > & neighbors, const FSVONodeAddress & leaf_address ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetLeafNeighbors );

    const MortonCode leaf_index = leaf_address.SubNodeIndex;
    const FSVONode & node = GetNodeFromAddress( leaf_address );
    const FSVOLeaf & leaf = SVOData.GetLeaves().GetLeaf( node.FirstChild.NodeIndex );

    uint_fast32_t x = 0, y = 0, z = 0;
    morton3D_64_decode( leaf_index, x, y, z );

    for ( NeighborDirection neighbor_direction = 0; neighbor_direction < 6; neighbor_direction++ )
    {
        FIntVector neighbor_coords( x, y, z );
        neighbor_coords += NeighborDirections[ neighbor_direction ];

        // If the neighbor is in bounds of this leaf node
        if ( neighbor_coords.X >= 0 && neighbor_coords.X < 4 && neighbor_coords.Y >= 0 && neighbor_coords.Y < 4 && neighbor_coords.Z >= 0 && neighbor_coords.Z < 4 )
        {
            const MortonCode subnode_index = FSVOHelpers::GetMortonCodeFromVector( neighbor_coords );
            // If this node is not blocked, this is a valid address, add it
            if ( !leaf.IsSubNodeOccluded( subnode_index ) )
            {
                neighbors.Emplace( FSVONodeAddress( 0, leaf_address.NodeIndex, subnode_index ) );
            }
        }
        else // the neighbor is out of bounds, we need to find our neighbor
        {
            const FSVONodeAddress & neighbor_address = node.Neighbors[ neighbor_direction ];
            const FSVONode & neighbor_node = GetNodeFromAddress( neighbor_address );

            // If the neighbor layer 0 has no leaf nodes, just return it
            if ( !neighbor_node.FirstChild.IsValid() )
            {
                neighbors.Add( neighbor_address );
                continue;
            }

            const FSVOLeaf & leaf_node = SVOData.GetLeaves().GetLeaf( neighbor_node.FirstChild.NodeIndex );

            // leaf not occluded. Find the correct subnode
            if ( !leaf_node.IsCompletelyOccluded() )
            {
                if ( neighbor_coords.X < 0 )
                {
                    neighbor_coords.X = 3;
                }
                else if ( neighbor_coords.X > 3 )
                {
                    neighbor_coords.X = 0;
                }
                else if ( neighbor_coords.Y < 0 )
                {
                    neighbor_coords.Y = 3;
                }
                else if ( neighbor_coords.Y > 3 )
                {
                    neighbor_coords.Y = 0;
                }
                else if ( neighbor_coords.Z < 0 )
                {
                    neighbor_coords.Z = 3;
                }
                else if ( neighbor_coords.Z > 3 )
                {
                    neighbor_coords.Z = 0;
                }

                const MortonCode subnode_index = FSVOHelpers::GetMortonCodeFromVector( neighbor_coords );

                // Only return the neighbor if it isn't blocked!
                if ( !leaf_node.IsSubNodeOccluded( subnode_index ) )
                {
                    neighbors.Emplace( FSVONodeAddress( 0, neighbor_node.FirstChild.NodeIndex, subnode_index ) );
                }
            }
            // else the leaf node is completely blocked, we don't return it
        }
    }
}

void FSVOVolumeNavigationData::GetFreeNodesFromNodeAddress( const FSVONodeAddress node_address, TArray< FSVONodeAddress > & free_nodes ) const
{
    const auto layer_index = node_address.LayerIndex;
    const auto node_index = node_address.NodeIndex;

    if ( layer_index == 0 )
    {
        const auto & leaf_node = SVOData.Leaves.GetLeaf( node_index );

        if ( leaf_node.IsCompletelyOccluded() )
        {
            return;
        }

        if ( leaf_node.IsCompletelyFree() )
        {
            free_nodes.Emplace( node_address );
            return;
        }

        for ( auto morton_code = 0; morton_code < 64; ++morton_code )
        {
            if ( !leaf_node.IsSubNodeOccluded( morton_code ) )
            {
                free_nodes.Emplace( FSVONodeAddress( 0, node_index, morton_code ) );
            }
        }
    }
    else
    {
        const auto & node = SVOData.GetLayer( layer_index ).GetNode( node_index );

        if ( !node.HasChildren() )
        {
            free_nodes.Emplace( node_address );
        }
        else
        {
            const auto & first_child = node.FirstChild;
            const auto child_layer_index = first_child.LayerIndex;
            const auto & child_layer = SVOData.GetLayer( child_layer_index );

            for ( auto child_index = 0; child_index < 8; ++child_index )
            {
                const auto & child_node = child_layer.GetNodes()[ first_child.NodeIndex + child_index ];
                GetFreeNodesFromNodeAddress( FSVONodeAddress( child_layer_index, child_node.MortonCode, 0 ), free_nodes );
            }
        }
    }
}

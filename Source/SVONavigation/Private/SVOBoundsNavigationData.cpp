#include "SVOBoundsNavigationData.h"

#include "SVONavigationTypes.h"

#include <libmorton/morton.h>

static const FIntVector NeighborDirections[ 6 ] = {
    { 1, 0, 0 },
    { -1, 0, 0 },
    { 0, 1, 0 },
    { 0, -1, 0 },
    { 0, 0, 1 },
    { 0, 0, -1 }
};

const NodeIndex ChildOffsetsDirections[ 6 ][ 4 ] = {
    { 0, 4, 2, 6 },
    { 1, 3, 5, 7 },
    { 0, 1, 4, 5 },
    { 2, 3, 6, 7 },
    { 0, 1, 2, 3 },
    { 4, 5, 6, 7 }
};

const NodeIndex LeafChildOffsetsDirections[ 6 ][ 16 ] = {
    { 0, 2, 16, 18, 4, 6, 20, 22, 32, 34, 48, 50, 36, 38, 52, 54 },
    { 9, 11, 25, 27, 13, 15, 29, 31, 41, 43, 57, 59, 45, 47, 61, 63 },
    { 0, 1, 8, 9, 4, 5, 12, 13, 32, 33, 40, 41, 36, 37, 44, 45 },
    { 18, 19, 26, 27, 22, 23, 30, 31, 50, 51, 58, 59, 54, 55, 62, 63 },
    { 0, 1, 8, 9, 2, 3, 10, 11, 16, 17, 24, 25, 18, 19, 26, 27 },
    { 36, 37, 44, 45, 38, 39, 46, 47, 52, 53, 60, 61, 54, 55, 62, 63 }

};

bool FSVOBoundsNavigationData::GetLinkFromPosition( FSVOOctreeLink & link, const FVector & position ) const
{
    if ( !NavigationBounds.IsInside( position ) )
    {
        return false;
    }

    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetLinkFromPosition );

    FVector origin;
    FVector extent;

    NavigationBounds.GetCenterAndExtents( origin, extent );
    // The z-order origin of the volume (where code == 0)
    FVector z_origin = origin - extent;
    // The local position of the point in volume space
    FVector local_position = position - z_origin;

    int layer_index = LayerCount - 1;
    NodeIndex nodeIndex = 0;

    while ( layer_index >= 0 && layer_index < LayerCount )
    {
        const auto & layer_nodes = GetLayerNodes( layer_index );
        const auto voxel_size = GetLayerVoxelSize( layer_index );

        FIntVector voxel_coords;
        voxel_coords.X = FMath::FloorToInt( ( local_position.X / voxel_size ) );
        voxel_coords.Y = FMath::FloorToInt( ( local_position.Y / voxel_size ) );
        voxel_coords.Z = FMath::FloorToInt( ( local_position.Z / voxel_size ) );

        // Get the morton code we want for this layer
        const auto code = morton3D_64_encode( voxel_coords.X, voxel_coords.Y, voxel_coords.Z );
        const auto half_voxel_size = GetLayerVoxelHalfSize( layer_index );

        for ( NodeIndex node_index = nodeIndex; node_index < layer_nodes.Num(); node_index++ )
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
                link.LayerIndex = layer_index;
                link.NodeIndex = node_index;
                link.SubNodeIndex = 0;
                return true;
            }

            // If this is a leaf node, we need to find our subnode
            if ( layer_index == 0 )
            {
                const auto & leaf = GetLeafNode( node.FirstChild.NodeIndex );
                // We need to calculate the node local position to get the morton code for the leaf
                // The world position of the 0 node
                const auto nodePosition = GetNodePosition( layer_index, node.MortonCode );
                // The morton origin of the node
                const auto nodeOrigin = nodePosition - FVector( half_voxel_size );
                // The requested position, relative to the node origin
                const auto nodeLocalPos = position - nodeOrigin;
                // Now get our voxel coordinates
                const auto voxel_quarter_size = voxel_size * 0.25f;

                FIntVector leaf_coords;
                leaf_coords.X = FMath::FloorToInt( nodeLocalPos.X / voxel_quarter_size );
                leaf_coords.Y = FMath::FloorToInt( nodeLocalPos.Y / voxel_quarter_size );
                leaf_coords.Z = FMath::FloorToInt( nodeLocalPos.Z / voxel_quarter_size );

                // So our link is.....*drum roll*
                link.LayerIndex = 0;         // Layer 0 (leaf)
                link.NodeIndex = node_index; // This index

                const auto leaf_code = morton3D_64_encode( leaf_coords.X, leaf_coords.Y, leaf_coords.Z ); // This morton code is our key into the 64-bit leaf node

                if ( leaf.GetSubNode( leaf_code ) )
                {
                    return false; // This voxel is blocked
                }

                link.SubNodeIndex = leaf_code;

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

bool FSVOBoundsNavigationData::GetLinkPosition( FVector & position, const FSVOOctreeLink & link ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetLinkPosition );
    
    const auto & layer_nodes = GetLayerNodes( link.LayerIndex );
    const auto & node = layer_nodes[ link.NodeIndex ];

    position = GetNodePosition( link.LayerIndex, node.MortonCode );

    if ( link.LayerIndex == 0 && node.FirstChild.IsValid() )
    {
        const auto voxel_size = GetLayerVoxelSize( 0 );

        uint_fast32_t x, y, z;
        morton3D_64_decode( link.SubNodeIndex, x, y, z );

        position += FVector( x * voxel_size * 0.25f, y * voxel_size * 0.25f, z * voxel_size * 0.25f ) - FVector( voxel_size * 0.375f );
        const auto & leaf_node = GetLeafNode( node.FirstChild.NodeIndex );
        const auto is_blocked = leaf_node.GetSubNode( link.SubNodeIndex );
        return !is_blocked;
    }

    return true;
}

void FSVOBoundsNavigationData::GetNeighbors( TArray< FSVOOctreeLink > & neighbors, const FSVOOctreeLink & link ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetNeighbors );
    
    const auto & link_node = GetNodeFromLink( link );
    if ( link.LayerIndex == 0 && link_node.FirstChild.IsValid() )
    {
        GetLeafNeighbors( neighbors, link );
        return;
    }

    const auto & node = GetNodeFromLink( link );

    for ( int neighbor_direction = 0; neighbor_direction < 6; neighbor_direction++ )
    {
        const auto & neighbor_link = node.Neighbors[ neighbor_direction ];

        if ( !neighbor_link.IsValid() )
        {
            continue;
        }

        const auto & neighbor = GetNodeFromLink( neighbor_link );

        // If the neighbour has no children, it's empty, we just use it
        if ( !neighbor.HasChildren() )
        {
            neighbors.Add( neighbor_link );
            continue;
        }

        // If the node has children, we need to look down the tree to see which children we want to add to the neighbour set

        // Start working set, and put the link into it
        TArray< FSVOOctreeLink > neighbor_links_working_set;
        neighbor_links_working_set.Push( neighbor_link );

        while ( neighbor_links_working_set.Num() > 0 )
        {
            // Pop off the top of the working set
            auto this_link = neighbor_links_working_set.Pop();

            const auto & this_node = GetNodeFromLink( this_link );

            // If the node as no children, it's clear, so add to neighbours and continue
            if ( !this_node.HasChildren() )
            {
                neighbors.Add( neighbor_link );
                continue;
            }

            // We know it has children

            if ( this_link.LayerIndex > 0 )
            {
                // If it's above layer 0, we will need to potentially add 4 children using our offsets
                for ( const auto & child_index : ChildOffsetsDirections[ neighbor_direction ] )
                {
                    // Each of the childnodes
                    auto child_link = this_node.FirstChild;
                    child_link.NodeIndex += child_index;
                    const auto & child_node = GetNodeFromLink( child_link );

                    if ( child_node.HasChildren() ) // If it has children, add them to the working set to keep going down
                    {
                        neighbor_links_working_set.Emplace( child_link );
                    }
                    else // Or just add to the outgoing links
                    {
                        neighbors.Emplace( child_link );
                    }
                }
            }
            else
            {
                // If this is a leaf layer, then we need to add whichever of the 16 facing leaf nodes aren't blocked
                for ( const auto & leaf_index : LeafChildOffsetsDirections[ neighbor_direction ] )
                {
                    // Each of the childnodes
                    auto child_link = neighbor.FirstChild;
                    const auto & leaf_node = GetLeafNode( child_link.NodeIndex );

                    child_link.SubNodeIndex = leaf_index;

                    if ( !leaf_node.GetSubNode( leaf_index ) )
                    {
                        neighbors.Emplace( child_link );
                    }
                }
            }
        }
    }
}

void FSVOBoundsNavigationData::GenerateNavigationData( const FBox & volume_bounds, const FSVOBoundsNavigationDataGenerationSettings & generation_settings )
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GenerateNavigationData );
    
    Settings = generation_settings;
    VolumeBounds = volume_bounds;

    const auto box_max_size = VolumeBounds.GetSize().GetAbsMax();
    const auto voxel_size = Settings.VoxelSize; // FMath::Max( NavDataConfig.AgentRadius, NavDataConfig.AgentHeight );

    VoxelExponent = FMath::RoundToInt( FMath::Log2( box_max_size / ( voxel_size * 4 ) ) );
    LayerCount = VoxelExponent + 1;

    const auto corrected_box_size = FMath::Pow( 2, VoxelExponent ) * ( voxel_size * 4 );
    const auto corrected_box_extent = corrected_box_size * 0.5f;

    NavigationBounds = FBox::BuildAABB( VolumeBounds.GetCenter(), FVector( corrected_box_extent ) );

    BlockedIndices.Reset();
    SVOData.Reset();

    LayerNodeCount.Reset( LayerCount );
    LayerVoxelSizes.Reset( LayerCount );
    LayerVoxelHalfSizes.Reset( LayerCount );

    for ( LayerIndex layer_index = 0; layer_index < LayerCount; ++layer_index )
    {
        LayerNodeCount.Add( FMath::Pow( FMath::Pow( 2, ( VoxelExponent - layer_index ) ), 3 ) );

        const auto layer_voxel_size = NavigationBounds.GetExtent().X / FMath::Pow( 2, VoxelExponent ) * FMath::Pow( 2.0f, layer_index + 1 );
        LayerVoxelSizes.Add( layer_voxel_size );
        LayerVoxelHalfSizes.Add( layer_voxel_size * 0.5f );
    }

    // Before we were checking if ( LayerCount < 2 ) but since LayerCount is unsigned it could wrap around
    if ( VoxelExponent < 1 )
    {
        return;
    }

    FirstPassRasterization();

    AllocateLeafNodes();

    RasterizeInitialLayer();

    for ( LayerIndex layer_index = 1; layer_index < LayerCount; ++layer_index )
    {
        RasterizeLayer( layer_index );
    }

    for ( LayerIndex layer_index = LayerCount - 2; layer_index != static_cast< LayerIndex >( -1 ); --layer_index )
    {
        BuildNeighborLinks( layer_index );
    }
}

FVector FSVOBoundsNavigationData::GetNodePosition( uint8 layer_index, MortonCode morton_code ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetNodePosition );
    
    const auto voxel_size = GetLayerVoxelSize( layer_index );
    uint_fast32_t x, y, z;
    morton3D_64_decode( morton_code, x, y, z );
    return NavigationBounds.GetCenter() - NavigationBounds.GetExtent() + FVector( x, y, z ) * voxel_size + FVector( voxel_size * 0.5f );
}

FVector FSVOBoundsNavigationData::GetNodePositionFromLink( const FSVOOctreeLink & link ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetNodePositionFromLink );
    
    const auto & node = SVOData.NodesByLayers[ link.LayerIndex ][ link.NodeIndex ];
    auto position = GetNodePosition( link.LayerIndex, node.MortonCode );

    if ( link.LayerIndex == 0 && node.FirstChild.IsValid() )
    {
        const float Size = GetLayerVoxelSize( 0 );
        uint_fast32_t X, Y, Z;
        morton3D_64_decode( link.SubNodeIndex, X, Y, Z );
        position += FVector( X * Size / 4, Y * Size / 4, Z * Size / 4 ) - FVector( Size * 0.375f );
    }

    return position;
}

bool FSVOBoundsNavigationData::IsPositionOccluded( const FVector & position, float box_size ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_IsPositionOccluded );
    return Settings.World->OverlapBlockingTestByChannel(
        position,
        FQuat::Identity,
        Settings.GenerationSettings.CollisionChannel,
        FCollisionShape::MakeBox( FVector( box_size + Settings.GenerationSettings.Clearance ) ),
        Settings.GenerationSettings.CollisionQueryParameters );
}

void FSVOBoundsNavigationData::FirstPassRasterization()
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_FirstPassRasterization );
    {
        auto & layer_blocked_indices = BlockedIndices.Emplace_GetRef();
        const auto node_count = GetLayerNodeCount( 1 );

        for ( MortonCode node_index = 0; node_index < node_count; ++node_index )
        {
            const auto position = GetNodePosition( 1, node_index );
            if ( IsPositionOccluded( position, GetLayerVoxelHalfSize( 1 ) ) )
            {
                layer_blocked_indices.Add( node_index );
            }
        }
    }

    {
        for ( int32 voxel_index = 0; voxel_index < VoxelExponent; voxel_index++ )
        {
            auto & layer_blocked_indices = BlockedIndices.Emplace_GetRef();
            for ( MortonCode morton_code : BlockedIndices[ voxel_index ] )
            {
                layer_blocked_indices.Add( morton_code >> 3 );
            }
        }
    }
}

void FSVOBoundsNavigationData::AllocateLeafNodes()
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_AllocateLeafNodes );
    SVOData.Leaves.AddDefaulted( BlockedIndices[ 0 ].Num() * 8 * 0.25f );
}

void FSVOBoundsNavigationData::RasterizeLeaf( const FVector & node_position, int32 leaf_index )
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_RasterizeLeaf );
    
    const auto layer_voxel_half_size = GetLayerVoxelHalfSize( 0 );
    const FVector Location = node_position - layer_voxel_half_size;
    const auto leaf_voxel_size = layer_voxel_half_size * 0.5f;
    const auto leaf_occlusion_voxel_size = leaf_voxel_size * 0.5f;

    for ( int32 subnode_index = 0; subnode_index < 64; subnode_index++ )
    {
        uint_fast32_t X, Y, Z;
        morton3D_64_decode( subnode_index, X, Y, Z );
        const FVector voxel_location = Location + FVector( X * leaf_voxel_size, Y * leaf_voxel_size, Z * leaf_voxel_size ) + leaf_voxel_size * 0.5f;

        if ( leaf_index >= SVOData.Leaves.Num() - 1 )
        {
            SVOData.Leaves.AddDefaulted( 1 );
        }

        if ( IsPositionOccluded( voxel_location, leaf_occlusion_voxel_size ) )
        {
            SVOData.Leaves[ leaf_index ].SetSubNode( subnode_index );
        }
    }
}

void FSVOBoundsNavigationData::RasterizeInitialLayer()
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_RasterizeInitialLayer );
    
    auto & layer_nodes = SVOData.NodesByLayers.Emplace_GetRef();
    int32 leaf_index = 0;

    SVOData.Leaves.Reserve( BlockedIndices[ 0 ].Num() * 8 );
    layer_nodes.Reserve( BlockedIndices[ 0 ].Num() * 8 );

    const auto layer_node_count = GetLayerNodeCount( 0 );

    for ( uint32 node_index = 0; node_index < layer_node_count; node_index++ )
    {
        // If we know this node needs to be added, from the low res first pass
        if ( !BlockedIndices[ 0 ].Contains( node_index >> 3 ) )
        {
            continue;
        }

        auto & octree_node = layer_nodes.Emplace_GetRef();
        octree_node.MortonCode = node_index;

        const auto node_position = GetNodePosition( 0, node_index );

        // Now check if we have any blocking, and search leaf nodes
        if ( IsPositionOccluded( node_position, GetLayerVoxelHalfSize( 0 ) ) )
        {
            RasterizeLeaf( node_position, leaf_index );
            octree_node.FirstChild.LayerIndex = 0;
            octree_node.FirstChild.NodeIndex = leaf_index;
            octree_node.FirstChild.SubNodeIndex = 0;
            leaf_index++;
        }
        else
        {
            SVOData.Leaves.AddDefaulted( 1 );
            leaf_index++;
            octree_node.FirstChild.Invalidate();
        }
    }
}

void FSVOBoundsNavigationData::RasterizeLayer( const LayerIndex layer_index )
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_RasterizeLayer );
    
    auto & layer_nodes = SVOData.NodesByLayers.Emplace_GetRef();

    checkf( layer_index > 0 && layer_index < LayerCount, TEXT( "layer_index is out of bounds" ) );

    layer_nodes.Reserve( BlockedIndices[ layer_index ].Num() * 8 );

    const auto node_count = GetLayerNodeCount( layer_index );

    for ( uint32 node_index = 0; node_index < node_count; node_index++ )
    {
        if ( !BlockedIndices[ layer_index ].Contains( node_index >> 3 ) )
        {
            continue;
        }

        const auto new_node_index = layer_nodes.Emplace();

        auto & new_octree_node = layer_nodes[ new_node_index ];
        new_octree_node.MortonCode = node_index;

        const auto child_index_from_code = GetNodeIndexFromMortonCode( layer_index - 1, new_octree_node.MortonCode << 3 );
        if ( child_index_from_code.IsSet() )
        {
            // Set parent->child links
            new_octree_node.FirstChild.LayerIndex = layer_index - 1;
            new_octree_node.FirstChild.NodeIndex = child_index_from_code.GetValue();

            // Set child->parent links
            for ( auto child_index = 0; child_index < 8; ++child_index )
            {
                auto & child_node = GetLayerNodes( new_octree_node.FirstChild.LayerIndex )[ new_octree_node.FirstChild.NodeIndex + child_index ];
                child_node.Parent.LayerIndex = layer_index;
                child_node.Parent.NodeIndex = new_node_index;
            }
        }
        else
        {
            new_octree_node.FirstChild.Invalidate();
        }
    }
}

TOptional< NodeIndex > FSVOBoundsNavigationData::GetNodeIndexFromMortonCode( const LayerIndex layer_index, const MortonCode morton_code ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetNodeIndexFromMortonCode );
    
    const auto & layer_nodes = GetLayerNodes( layer_index );
    auto start = 0;
    auto end = layer_nodes.Num() - 1;
    auto mean = ( start + end ) * 0.5f;

    // Binary search by Morton code
    while ( start <= end )
    {
        if ( layer_nodes[ mean ].MortonCode < morton_code )
        {
            start = mean + 1;
        }
        else if ( layer_nodes[ mean ].MortonCode == morton_code )
        {
            return mean;
        }
        else
        {
            end = mean - 1;
        }

        mean = ( start + end ) * 0.5f;
    }

    return TOptional< NodeIndex >();
}

void FSVOBoundsNavigationData::BuildNeighborLinks( LayerIndex layer_index )
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_BuildNeighborLinks );
    
    auto & layer_nodes = GetLayerNodes( layer_index );

    for ( NodeIndex layer_node_index = 0; layer_node_index < layer_nodes.Num(); layer_node_index++ )
    {
        auto & node = layer_nodes[ layer_node_index ];
        FVector node_position = GetNodePosition( layer_index, node.MortonCode );

        for ( NeighborDirection direction = 0; direction < 6; direction++ )
        {
            NodeIndex node_index = layer_node_index;

            FSVOOctreeLink & link = node.Neighbors[ direction ];

            LayerIndex current_layer = layer_index;

            while ( !FindNeighborInDirection( link, current_layer, node_index, direction, node_position ) && current_layer < LayerCount - 2 )
            {
                auto & parent_node = GetLayerNodes( current_layer )[ node_index ].Parent;
                if ( parent_node.IsValid() )
                {
                    node_index = parent_node.NodeIndex;
                    current_layer = parent_node.LayerIndex;
                }
                else
                {
                    current_layer++;
                    node_index = GetNodeIndexFromMortonCode( current_layer, node.MortonCode >> 3 ).Get( 0 );
                }
            }
        }
    }
}

bool FSVOBoundsNavigationData::FindNeighborInDirection( FSVOOctreeLink & link, const LayerIndex layer_index, const NodeIndex node_index, const NeighborDirection direction, const FVector & node_position )
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_FindNeighborInDirection );
    
    const auto max_coordinates = GetLayerMaxNodeCount( layer_index );
    const auto & layer_nodes = GetLayerNodes( layer_index );
    const auto & target_node = layer_nodes[ node_index ];

    uint_fast32_t x, y, z;
    morton3D_64_decode( target_node.MortonCode, x, y, z );

    FIntVector neighbor_coords( static_cast< int32 >( x ), static_cast< int32 >( y ), static_cast< int32 >( z ) );
    neighbor_coords += NeighborDirections[ direction ];

    if ( neighbor_coords.X < 0 || neighbor_coords.X >= max_coordinates ||
         neighbor_coords.Y < 0 || neighbor_coords.Y >= max_coordinates ||
         neighbor_coords.Z < 0 || neighbor_coords.Z >= max_coordinates )
    {
        link.Invalidate();
        return true;
    }

    x = neighbor_coords.X;
    y = neighbor_coords.Y;
    z = neighbor_coords.Z;

    const MortonCode neighbor_code = morton3D_64_encode( x, y, z );

    int32 stop_index = layer_nodes.Num();
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
                 SVOData.Leaves[ node.FirstChild.NodeIndex ].IsOccluded() )
            {
                link.Invalidate();
                return true;
            }

            link.LayerIndex = layer_index;

            if ( neighbor_node_index >= layer_nodes.Num() || neighbor_node_index < 0 )
            {
                break;
            }

            link.NodeIndex = neighbor_node_index;

            /*FVector AdjacentLocation;
            GetNodeLocation( LayerIndex, AdjacentCode, AdjacentLocation );*/

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

void FSVOBoundsNavigationData::GetLeafNeighbors( TArray< FSVOOctreeLink > & neighbors, const FSVOOctreeLink & link ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetLeafNeighbors );

    MortonCode leaf_index = link.SubNodeIndex;
    const FSVOOctreeNode & node = GetNodeFromLink( link );
    const FSVOOctreeLeaf & leaf = GetLeafNode( node.FirstChild.NodeIndex );

    // Get our starting co-ordinates
    uint_fast32_t x = 0, y = 0, z = 0;
    morton3D_64_decode( leaf_index, x, y, z );

    for ( int neighbor_direction = 0; neighbor_direction < 6; neighbor_direction++ )
    {
        // Need to switch to signed ints
        FIntVector neighbor_coords( static_cast< int32 >( x ), static_cast< int32 >( y ), static_cast< int32 >( z ) );
        neighbor_coords += NeighborDirections[ neighbor_direction ];

        // If the neighbour is in bounds of this leaf node
        if ( neighbor_coords.X >= 0 && neighbor_coords.X < 4 && neighbor_coords.Y >= 0 && neighbor_coords.Y < 4 && neighbor_coords.Z >= 0 && neighbor_coords.Z < 4 )
        {
            MortonCode subnode_index = morton3D_64_encode( neighbor_coords.X, neighbor_coords.Y, neighbor_coords.Z );
            // If this node is not blocked, his is a valid link, add it. Otherwise, no link in this direction, continue
            if ( !leaf.GetSubNode( subnode_index ) )
            {
                neighbors.Emplace( FSVOOctreeLink( 0, link.NodeIndex, subnode_index ) );
            }
        }
        else // the neighbor is out of bounds, we need to find our neighbor
        {
            const FSVOOctreeLink & neighbor_link = node.Neighbors[ neighbor_direction ];
            const FSVOOctreeNode & neighbor_node = GetNodeFromLink( neighbor_link );

            // If the neighbour layer 0 has no leaf nodes, just return it
            if ( !neighbor_node.FirstChild.IsValid() )
            {
                neighbors.Add( neighbor_link );
                continue;
            }

            const FSVOOctreeLeaf & leaf_node = GetLeafNode( neighbor_node.FirstChild.NodeIndex );

            // leaf not occluded. Find the correct subnode
            if ( !leaf_node.IsOccluded() )
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

                const auto subnode_code = morton3D_64_encode( neighbor_coords.X, neighbor_coords.Y, neighbor_coords.Z );

                // Only return the neighbour if it isn't blocked!
                if ( !leaf_node.GetSubNode( subnode_code ) )
                {
                    neighbors.Emplace( FSVOOctreeLink( 0, neighbor_node.FirstChild.NodeIndex, subnode_code ) );
                }
            }
            // else the leaf node is completely blocked, we don't return it
        }
    }
}

#include "SVOBoundsNavigationData.h"

#include "SVOHelpers.h"
#include "SVONavigationData.h"
#include "SVONavigationTypes.h"
#include "SVOVersion.h"

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
    const auto & navigation_bounds = SVOData.GetNavigationBounds();

    if ( !navigation_bounds.IsInside( position ) )
    {
        return false;
    }

    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetLinkFromPosition );

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
                link.LayerIndex = layer_index;
                link.NodeIndex = node_index;
                link.SubNodeIndex = 0;
                return true;
            }

            // If this is a leaf node, we need to find our subnode
            if ( layer_index == 0 )
            {
                const auto & leaves = SVOData.GetLeaves();
                const auto & leaf = leaves.GetLeaf( node.FirstChild.NodeIndex );

                // We need to calculate the node local position to get the morton code for the leaf
                // The world position of the 0 node
                const auto node_position = GetNodePosition( layer_index, node.MortonCode );
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

                link.LayerIndex = 0;
                link.NodeIndex = node_index;

                const auto leaf_code = FSVOHelpers::GetMortonCodeFromVector( leaf_coords ); // This morton code is our key into the 64-bit leaf node

                if ( leaf.IsSubNodeOccluded( leaf_code ) )
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

    for ( NeighborDirection neighbor_direction = 0; neighbor_direction < 6; neighbor_direction++ )
    {
        const auto & neighbor_link = node.Neighbors[ neighbor_direction ];

        if ( !neighbor_link.IsValid() )
        {
            continue;
        }

        const auto & neighbor = GetNodeFromLink( neighbor_link );

        // If the neighbor has no children, it's empty, we just use it
        if ( !neighbor.HasChildren() )
        {
            neighbors.Add( neighbor_link );
            continue;
        }

        // If the node has children, we need to look down the tree to see which children we want to add to the neighbor set

        // Start working set, and put the link into it
        TArray< FSVOOctreeLink > neighbor_links_working_set;
        neighbor_links_working_set.Push( neighbor_link );

        while ( neighbor_links_working_set.Num() > 0 )
        {
            // Pop off the top of the working set
            auto this_link = neighbor_links_working_set.Pop();

            const auto & this_node = GetNodeFromLink( this_link );

            // If the node as no children, it's clear, so add to neighbors and continue
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
                    const auto & leaf_node = SVOData.GetLeaves().GetLeaf( child_link.NodeIndex );

                    child_link.SubNodeIndex = leaf_index;

                    if ( !leaf_node.IsSubNodeOccluded( leaf_index ) )
                    {
                        neighbors.Emplace( child_link );
                    }
                }
            }
        }
    }
}

float FSVOBoundsNavigationData::GetLayerRatio( const LayerIndex layer_index ) const
{
    return static_cast< float >( layer_index ) / GetLayerCount();
}

float FSVOBoundsNavigationData::GetLayerInverseRatio( const LayerIndex layer_index ) const
{
    return 1.0f - GetLayerRatio( layer_index );
}

float FSVOBoundsNavigationData::GetVoxelHalfExtentFromLink( FSVOOctreeLink link ) const
{
    if ( link.LayerIndex == 0 )
    {
        return SVOData.GetLeaves().GetLeafSubNodeHalfExtent();
    }

    return SVOData.GetLayer( link.LayerIndex ).GetVoxelHalfExtent();
}

void FSVOBoundsNavigationData::GenerateNavigationData( const FBox & volume_bounds, const FSVOBoundsNavigationDataGenerationSettings & generation_settings )
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
}

void FSVOBoundsNavigationData::Serialize( FArchive & archive, const ESVOVersion /*version*/ )
{
    archive << VolumeBounds;
    archive << SVOData;
}

FVector FSVOBoundsNavigationData::GetNodePosition( const LayerIndex layer_index, const MortonCode morton_code ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetNodePosition );

    const auto & layer = SVOData.GetLayer( layer_index );
    const auto voxel_size = layer.GetVoxelExtent();
    const auto voxel_half_size = layer.GetVoxelHalfExtent();
    const auto morton_coords = FSVOHelpers::GetVectorFromMortonCode( morton_code );
    const auto & navigation_bounds = SVOData.GetNavigationBounds();

    return navigation_bounds.GetCenter() - navigation_bounds.GetExtent() + morton_coords * voxel_size + voxel_half_size;
}

FVector FSVOBoundsNavigationData::GetLinkPosition( const FSVOOctreeLink & link ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetNodePositionFromLink );

    const auto & layer = SVOData.GetLayer( link.LayerIndex );

    const auto & node = layer.GetNode( link.NodeIndex );
    auto position = GetNodePosition( link.LayerIndex, node.MortonCode );

    if ( link.LayerIndex == 0 && node.FirstChild.IsValid() )
    {
        const auto & leaves = SVOData.GetLeaves();
        const auto leaf_extent = leaves.GetLeafExtent();
        const auto leaf_subnode_extent = SVOData.GetLeaves().GetLeafSubNodeExtent();
        const auto morton_coords = FSVOHelpers::GetVectorFromMortonCode( link.SubNodeIndex );

        position += morton_coords * leaf_subnode_extent - leaf_extent * 0.375f;
    }

    return position;
}

bool FSVOBoundsNavigationData::IsPositionOccluded( const FVector & position, const float box_half_extent ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_IsPositionOccluded );
    return Settings.World->OverlapBlockingTestByChannel(
        position,
        FQuat::Identity,
        Settings.GenerationSettings.CollisionChannel,
        FCollisionShape::MakeBox( FVector( box_half_extent + Settings.GenerationSettings.Clearance ) ),
        Settings.GenerationSettings.CollisionQueryParameters );
}

void FSVOBoundsNavigationData::FirstPassRasterization()
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_FirstPassRasterization );
    {
        const auto & layer = SVOData.GetLayer( 1 );
        const auto layer_max_node_count = layer.GetMaxNodeCount();
        const auto layer_voxel_half_extent = layer.GetVoxelHalfExtent();

        auto & layer_zero = SVOData.GetLayer( 0 );

        for ( MortonCode node_index = 0; node_index < layer_max_node_count; ++node_index )
        {
            const auto position = GetNodePosition( 1, node_index );

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

void FSVOBoundsNavigationData::RasterizeLeaf( const FVector & node_position, const LeafIndex leaf_index )
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

void FSVOBoundsNavigationData::RasterizeInitialLayer()
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
        const auto is_blocked = layer_zero.GetBlockedNodes().Contains( FSVOHelpers::GetParentMortonCode( node_index ) );

        // If we know this node needs to be added, from the low res first pass
        if ( !is_blocked )
        {
            continue;
        }

        auto & layer_zero_node = layer_zero_nodes.Emplace_GetRef();
        layer_zero_node.MortonCode = node_index;

        const auto node_position = GetNodePosition( 0, node_index );

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

void FSVOBoundsNavigationData::RasterizeLayer( const LayerIndex layer_index )
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
        const auto child_index_from_code = GetNodeIndexFromMortonCode( child_layer_index, FSVOHelpers::GetFirstChildMortonCode( layer_node.MortonCode ) );

        if ( child_index_from_code.IsSet() )
        {
            // Set parent->child links
            layer_node.FirstChild.LayerIndex = child_layer_index;
            layer_node.FirstChild.NodeIndex = child_index_from_code.GetValue();

            auto & child_layer = SVOData.GetLayer( layer_node.FirstChild.LayerIndex );

            // Set child->parent links
            for ( auto child_index = 0; child_index < 8; ++child_index )
            {
                auto & child_node_layer = child_layer;
                auto & child_node = child_node_layer.GetNodes()[ layer_node.FirstChild.NodeIndex + child_index ];

                child_node.Parent.LayerIndex = layer_index;
                child_node.Parent.NodeIndex = new_node_index;
            }
        }
        else
        {
            layer_node.FirstChild.Invalidate();
        }
    }
}

TOptional< NodeIndex > FSVOBoundsNavigationData::GetNodeIndexFromMortonCode( const LayerIndex layer_index, const MortonCode morton_code ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetNodeIndexFromMortonCode );

    const auto & layer_nodes = SVOData.GetLayer( layer_index ).GetNodes();
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

void FSVOBoundsNavigationData::BuildNeighborLinks( const LayerIndex layer_index )
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_BuildNeighborLinks );

    auto & layer_nodes = SVOData.GetLayer( layer_index ).GetNodes();
    const auto max_layer_index = GetLayerCount() - 2;

    for ( NodeIndex layer_node_index = 0; layer_node_index < static_cast< uint32 >( layer_nodes.Num() ); layer_node_index++ )
    {
        auto & node = layer_nodes[ layer_node_index ];
        FVector node_position = GetNodePosition( layer_index, node.MortonCode );

        for ( NeighborDirection direction = 0; direction < 6; direction++ )
        {
            NodeIndex node_index = layer_node_index;
            FSVOOctreeLink & link = node.Neighbors[ direction ];
            LayerIndex current_layer = layer_index;

            while ( !FindNeighborInDirection( link, current_layer, node_index, direction, node_position ) && current_layer < max_layer_index )
            {
                auto & parent_link = SVOData.GetLayer( current_layer ).GetNodes()[ node_index ].Parent;
                if ( parent_link.IsValid() )
                {
                    node_index = parent_link.NodeIndex;
                    current_layer = parent_link.LayerIndex;
                }
                else
                {
                    current_layer++;
                    node_index = GetNodeIndexFromMortonCode( current_layer, FSVOHelpers::GetParentMortonCode( node.MortonCode ) ).Get( 0 );
                }
            }
        }
    }
}

bool FSVOBoundsNavigationData::FindNeighborInDirection( FSVOOctreeLink & link, const LayerIndex layer_index, const NodeIndex node_index, const NeighborDirection direction, const FVector & node_position )
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
        link.Invalidate();
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
                link.Invalidate();
                return true;
            }

            link.LayerIndex = layer_index;

            if ( neighbor_node_index >= layer_nodes_count || neighbor_node_index < 0 )
            {
                break;
            }

            link.NodeIndex = neighbor_node_index;

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

    const MortonCode leaf_index = link.SubNodeIndex;
    const FSVOOctreeNode & node = GetNodeFromLink( link );
    const FSVOOctreeLeaf & leaf = SVOData.GetLeaves().GetLeaf( node.FirstChild.NodeIndex );

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
            // If this node is not blocked, this is a valid link, add it. Otherwise, no link in this direction, continue
            if ( !leaf.IsSubNodeOccluded( subnode_index ) )
            {
                neighbors.Emplace( FSVOOctreeLink( 0, link.NodeIndex, subnode_index ) );
            }
        }
        else // the neighbor is out of bounds, we need to find our neighbor
        {
            const FSVOOctreeLink & neighbor_link = node.Neighbors[ neighbor_direction ];
            const FSVOOctreeNode & neighbor_node = GetNodeFromLink( neighbor_link );

            // If the neighbor layer 0 has no leaf nodes, just return it
            if ( !neighbor_node.FirstChild.IsValid() )
            {
                neighbors.Add( neighbor_link );
                continue;
            }

            const FSVOOctreeLeaf & leaf_node = SVOData.GetLeaves().GetLeaf( neighbor_node.FirstChild.NodeIndex );

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
                    neighbors.Emplace( FSVOOctreeLink( 0, neighbor_node.FirstChild.NodeIndex, subnode_index ) );
                }
            }
            // else the leaf node is completely blocked, we don't return it
        }
    }
}
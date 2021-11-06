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
                const auto & leaf = SVOData.GetLeaves().GetLeaf( node.FirstChild.NodeIndex );
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

                link.LayerIndex = 0;
                link.NodeIndex = node_index;

                const auto leaf_code = FSVOHelpers::GetMortonCodeFromVector( leaf_coords ); // This morton code is our key into the 64-bit leaf node

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

                    if ( !leaf_node.GetSubNode( leaf_index ) )
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
        SVOData.GetLeaves().AllocateLeaves( BlockedIndices[ 0 ].Num() * 8 );
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

void FSVOBoundsNavigationData::Serialize( FArchive & archive, const ESVOVersion version )
{
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

    const auto & node = SVOData.GetLayer( link.LayerIndex ).GetNode( link.NodeIndex );
    auto position = GetNodePosition( link.LayerIndex, node.MortonCode );

    if ( link.LayerIndex == 0 && node.FirstChild.IsValid() )
    {
        const auto & layer = SVOData.GetLayer( 0 );
        const auto voxel_size = layer.GetVoxelExtent();
        const auto morton_coords = FSVOHelpers::GetVectorFromMortonCode( link.SubNodeIndex );
        position += morton_coords * voxel_size / 4 - voxel_size * 0.375f;
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
        auto & layer_blocked_indices = BlockedIndices.Emplace_GetRef();
        const auto & layer = SVOData.GetLayer( 1 );
        const auto node_count = layer.GetMaxNodeCount();

        for ( MortonCode node_index = 0; node_index < node_count; ++node_index )
        {
            const auto position = GetNodePosition( 1, node_index );
            if ( IsPositionOccluded( position, layer.GetVoxelHalfExtent() ) )
            {
                layer_blocked_indices.Add( node_index );
            }
        }
    }

    {
        for ( int32 voxel_index = 0; voxel_index < GetLayerCount() - 1; voxel_index++ )
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
    
}

void FSVOBoundsNavigationData::RasterizeLeaf( const FVector & node_position, const LeafIndex leaf_index )
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_RasterizeLeaf );

    const auto & layer = SVOData.GetLayer( 0 );
    const auto layer_voxel_half_extent = layer.GetVoxelHalfExtent();
    const auto location = node_position - layer_voxel_half_extent;
    const auto leaf_half_extent = layer_voxel_half_extent * 0.5f;
    const auto leaf_subnode_half_extent = leaf_half_extent * 0.5f;

    for ( SubNodeIndex subnode_index = 0; subnode_index < 64; subnode_index++ )
    {
        const auto morton_coords = FSVOHelpers::GetVectorFromMortonCode( subnode_index );
        const auto voxel_location = location + morton_coords * leaf_half_extent + leaf_half_extent * 0.5f;

        SVOData.GetLeaves().AddLeaf( leaf_index, subnode_index, IsPositionOccluded( voxel_location, leaf_subnode_half_extent ) );
    }
}

void FSVOBoundsNavigationData::RasterizeInitialLayer()
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_RasterizeInitialLayer );

    auto & layer = SVOData.GetLayer( 0 );
    auto & layer_nodes = layer.GetNodes();
    LeafIndex leaf_index = 0;

    layer_nodes.Reserve( BlockedIndices[ 0 ].Num() * 8 );

    const auto layer_max_node_count = layer.GetMaxNodeCount();
    const auto layer_voxel_half_size = layer.GetVoxelHalfExtent();

    for ( NodeIndex node_index = 0; node_index < layer_max_node_count; node_index++ )
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
        if ( IsPositionOccluded( node_position, layer_voxel_half_size ) )
        {
            RasterizeLeaf( node_position, leaf_index );
            octree_node.FirstChild.LayerIndex = 0;
            octree_node.FirstChild.NodeIndex = leaf_index;
            octree_node.FirstChild.SubNodeIndex = 0;
            leaf_index++;
        }
        else
        {
            SVOData.GetLeaves().AddEmptyLeaf();
            leaf_index++;
            octree_node.FirstChild.Invalidate();
        }
    }
}

void FSVOBoundsNavigationData::RasterizeLayer( const LayerIndex layer_index )
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_RasterizeLayer );

    auto & layer = SVOData.GetLayer( layer_index );
    auto & layer_nodes = layer.GetNodes();

    checkf( layer_index > 0 && layer_index < GetLayerCount(), TEXT( "layer_index is out of bounds" ) );

    layer_nodes.Reserve( BlockedIndices[ layer_index ].Num() * 8 );

    const auto layer_max_node_count = layer.GetMaxNodeCount();

    for ( NodeIndex node_index = 0; node_index < layer_max_node_count; node_index++ )
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
                auto & child_node_layer = SVOData.GetLayer( new_octree_node.FirstChild.LayerIndex );
                auto & child_node = child_node_layer.GetNodes()[ new_octree_node.FirstChild.NodeIndex + child_index ];
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

    for ( NodeIndex layer_node_index = 0; layer_node_index < static_cast< uint32 >( layer_nodes.Num() ); layer_node_index++ )
    {
        auto & node = layer_nodes[ layer_node_index ];
        FVector node_position = GetNodePosition( layer_index, node.MortonCode );

        for ( NeighborDirection direction = 0; direction < 6; direction++ )
        {
            NodeIndex node_index = layer_node_index;

            FSVOOctreeLink & link = node.Neighbors[ direction ];

            LayerIndex current_layer = layer_index;

            const auto max_layer_index = GetLayerCount() - 2;

            while ( !FindNeighborInDirection( link, current_layer, node_index, direction, node_position ) && current_layer < max_layer_index )
            {
                auto & parent_node = SVOData.GetLayer( current_layer ).GetNodes()[ node_index ].Parent;
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

    const auto max_coordinates = static_cast< int32 >( SVOData.GetLayer( layer_index ).GetMaxNodeCount() );
    const auto & layer_nodes = SVOData.GetLayer( layer_index ).GetNodes();
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
                 SVOData.GetLeaves().GetLeaf( node.FirstChild.NodeIndex ).IsOccluded() )
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

    for ( int neighbor_direction = 0; neighbor_direction < 6; neighbor_direction++ )
    {
        FIntVector neighbor_coords( x, y, z );
        neighbor_coords += NeighborDirections[ neighbor_direction ];

        // If the neighbor is in bounds of this leaf node
        if ( neighbor_coords.X >= 0 && neighbor_coords.X < 4 && neighbor_coords.Y >= 0 && neighbor_coords.Y < 4 && neighbor_coords.Z >= 0 && neighbor_coords.Z < 4 )
        {
            const MortonCode subnode_index = FSVOHelpers::GetMortonCodeFromVector( neighbor_coords );
            // If this node is not blocked, this is a valid link, add it. Otherwise, no link in this direction, continue
            if ( !leaf.GetSubNode( subnode_index ) )
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

                const MortonCode subnode_index = FSVOHelpers::GetMortonCodeFromVector( neighbor_coords );

                // Only return the neighbor if it isn't blocked!
                if ( !leaf_node.GetSubNode( subnode_index ) )
                {
                    neighbors.Emplace( FSVOOctreeLink( 0, neighbor_node.FirstChild.NodeIndex, subnode_index ) );
                }
            }
            // else the leaf node is completely blocked, we don't return it
        }
    }
}
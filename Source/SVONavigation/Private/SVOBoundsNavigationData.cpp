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

void FSVOBoundsNavigationData::GenerateNavigationData( const FBox & volume_bounds, const FSVOBoundsNavigationDataGenerationSettings & generation_settings )
{
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
    const auto voxel_size = GetLayerVoxelSize( layer_index );
    uint_fast32_t x, y, z;
    morton3D_64_decode( morton_code, x, y, z );
    return NavigationBounds.GetCenter() - NavigationBounds.GetExtent() + FVector( x, y, z ) * voxel_size + FVector( voxel_size * 0.5f );
}

FVector FSVOBoundsNavigationData::GetNodePositionFromLink( const FSVOOctreeLink & link ) const
{
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
    return Settings.World->OverlapBlockingTestByChannel(
        position,
        FQuat::Identity,
        Settings.GenerationSettings.CollisionChannel,
        FCollisionShape::MakeBox( FVector( box_size + Settings.GenerationSettings.Clearance ) ),
        Settings.GenerationSettings.CollisionQueryParameters );
}

void FSVOBoundsNavigationData::FirstPassRasterization()
{
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
    SVOData.Leaves.AddDefaulted( BlockedIndices[ 0 ].Num() * 8 * 0.25f );
}

void FSVOBoundsNavigationData::RasterizeLeaf( const FVector & node_position, int32 leaf_index )
{
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
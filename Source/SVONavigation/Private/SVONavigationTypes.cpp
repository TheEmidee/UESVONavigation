#include "SVONavigationTypes.h"

#include "SVOPathFindingAlgorithm.h"
#include "SVOTraversalCostCalculator.h"

#include <libmorton/morton.h>

bool FSVOOctreeLeaf::GetSubNodeAt( uint_fast32_t x, uint_fast32_t y, uint_fast32_t z ) const
{
    constexpr uint_fast64_t MortonCode = 0;
    morton3D_64_decode( MortonCode, x, y, z );
    return ( SubNodes & 1ULL << morton3D_64_encode( x, y, z ) ) != 0;
}

void FSVOOctreeLeaf::SetSubNodeAt( uint_fast32_t x, uint_fast32_t y, uint_fast32_t z )
{
    constexpr uint_fast64_t MortonCode = 0;
    morton3D_64_decode( MortonCode, x, y, z );
    SubNodes |= 1ULL << morton3D_64_encode( x, y, z );
}

void FSVOLeaves::Initialize( const float leaf_extent )
{
    LeafExtent = leaf_extent;
}

void FSVOLeaves::Reset()
{
    Leaves.Reset();
}

int FSVOLeaves::GetAllocatedSize() const
{
    return Leaves.Num() * sizeof( FSVOOctreeLeaf );
}

void FSVOLeaves::AllocateLeaves( const int leaf_count )
{
    Leaves.Reserve( leaf_count );
}

void FSVOLeaves::AddLeaf( const LeafIndex leaf_index, const SubNodeIndex subnode_index, const bool is_occluded )
{
    if ( leaf_index >= Leaves.Num() - 1 )
    {
        AddEmptyLeaf();
    }

    if ( is_occluded )
    {
        Leaves[ leaf_index ].SetSubNode( subnode_index );
    }
}

void FSVOLeaves::AddEmptyLeaf()
{
    Leaves.AddDefaulted( 1 );
}

FSVOLayer::FSVOLayer() :
    MaxNodeCount( -1 ),
    VoxelExtent( 0.0f ),
    VoxelHalfExtent( 0.0f )
{
}

FSVOLayer::FSVOLayer( const int max_node_count, const float voxel_extent ) :
    MaxNodeCount( max_node_count ),
    VoxelExtent( voxel_extent ),
    VoxelHalfExtent( voxel_extent * 0.5f )
{
}

int FSVOLayer::GetAllocatedSize() const
{
    return Nodes.Num() * sizeof( FSVOOctreeNode );
}

bool FSVOOctreeData::Initialize( const float voxel_extent, const FBox & volume_bounds )
{
    Reset();

    const auto volume_extent = volume_bounds.GetSize().GetAbsMax();

    const auto leaf_extent = voxel_extent * 4;
    const auto voxel_exponent = FMath::CeilToInt( FMath::Log2( volume_extent / leaf_extent ) );
    const auto layer_count = voxel_exponent + 1;

    if ( layer_count < 2 )
    {
        return false;
    }

    Leaves.Initialize( leaf_extent );

    const auto navigation_bounds_extent = FMath::Pow( 2, voxel_exponent ) * leaf_extent;

    for ( LayerIndex layer_index = 0; layer_index < layer_count; ++layer_index )
    {
        const auto layer_edge_node_count = FMath::Pow( 2, voxel_exponent - layer_index );
        const auto layer_max_node_count = FMath::CeilToInt( FMath::Pow( layer_edge_node_count, 3 ) );
        const auto layer_voxel_size = navigation_bounds_extent / layer_edge_node_count;

        Layers.Emplace( layer_max_node_count, layer_voxel_size );
    }

    // The second parameter of FBox::BuildAABB is named Extent but it's really the half extent
    NavigationBounds = FBox::BuildAABB( volume_bounds.GetCenter(), FVector( navigation_bounds_extent * 0.5f ) );

    return true;
}

void FSVOOctreeData::Reset()
{
    Layers.Reset();
    Leaves.Reset();
}

int FSVOOctreeData::GetAllocatedSize() const
{
    int size = Leaves.GetAllocatedSize();

    for ( const auto & layer : Layers )
    {
        size += layer.GetAllocatedSize();
    }

    return size;
}

FSVONavigationQueryFilterSettings::FSVONavigationQueryFilterSettings() :
    PathFinder( nullptr ),
    TraversalCostCalculator( nullptr ),
    HeuristicCalculator( nullptr ),
    HeuristicScale( 1.0f ),
    bUseNodeSizeCompensation( true ),
    bOffsetPathVerticallyByAgentRadius( true )
{
}

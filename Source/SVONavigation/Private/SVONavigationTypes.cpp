#include "SVONavigationTypes.h"

#include "SVOPathFindingAlgorithm.h"
#include "SVOTraversalCostCalculator.h"

#include <libmorton/morton.h>

bool FSVOOctreeLeaf::GetSubNodeAt( uint_fast32_t X, uint_fast32_t Y, uint_fast32_t Z ) const
{
    constexpr uint_fast64_t MortonCode = 0;
    morton3D_64_decode( MortonCode, X, Y, Z );
    return ( SubNodes & 1ULL << morton3D_64_encode( X, Y, Z ) ) != 0;
}

void FSVOOctreeLeaf::SetSubNodeAt( uint_fast32_t X, uint_fast32_t Y, uint_fast32_t Z )
{
    constexpr uint_fast64_t MortonCode = 0;
    morton3D_64_decode( MortonCode, X, Y, Z );
    SubNodes |= 1ULL << morton3D_64_encode( X, Y, Z );
}

FSVOLayer::FSVOLayer() :
    MaxNodeCount( -1 ),
    VoxelSize( 0.0f ),
    VoxelHalfSize( 0.0f )
{
}

FSVOLayer::FSVOLayer( const int max_node_count, const float voxel_size ) :
    MaxNodeCount( max_node_count ),
    VoxelSize( voxel_size ),
    VoxelHalfSize( voxel_size * 0.5f )
{
}

int FSVOLayer::GetAllocatedSize() const
{
    return Nodes.Num() * sizeof( FSVOOctreeNode );
}

void FSVOOctreeData::Reset()
{
    Layers.Reset();
    Leaves.Reset();
}

int FSVOOctreeData::GetAllocatedSize() const
{
    int size = Leaves.Num() * sizeof( FSVOOctreeLeaf );

    for ( const auto & layer : Layers )
    {
        size += layer.GetAllocatedSize();
    }

    return size;
}

void FSVOOctreeData::AllocateLeafNodes( const int leaf_count )
{
    Leaves.Reserve( leaf_count );
}

void FSVOOctreeData::AddLeaf( const LeafIndex leaf_index, const SubNodeIndex subnode_index, const bool is_occluded )
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

void FSVOOctreeData::AddEmptyLeaf()
{
    Leaves.AddDefaulted( 1 );
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

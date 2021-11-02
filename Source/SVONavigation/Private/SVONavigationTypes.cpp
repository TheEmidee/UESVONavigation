#include "SVONavigationTypes.h"

#include "SVOTraversalCostCalculator.h"
#include "SVOPathFindingAlgorithm.h"

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

void FSVOOctreeData::Reset()
{
    NodesByLayers.Reset();
    Leaves.Reset();
}

int FSVOOctreeData::GetAllocatedSize() const
{
    int size = Leaves.Num() * sizeof( FSVOOctreeLeaf );

    for ( const auto & layer_nodes : NodesByLayers )
    {
        size += layer_nodes.Num() * sizeof( FSVOOctreeNode );
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

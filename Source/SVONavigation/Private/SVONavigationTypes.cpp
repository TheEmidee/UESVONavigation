#include "SVONavigationTypes.h"

#include "PathFinding/SVOPathFindingAlgorithm.h"

const FSVONodeAddress FSVONodeAddress::InvalidAddress;

void FSVOLeafNodes::Initialize( const float leaf_size )
{
    LeafNodeSize = leaf_size;
}

void FSVOLeafNodes::Reset()
{
    LeafNodes.Reset();
}

FSVONode::FSVONode() :
    MortonCode( 0 ),
    Parent( FSVONodeAddress::InvalidAddress ),
    FirstChild( FSVONodeAddress::InvalidAddress )
{
}

FSVONode::FSVONode( const ::MortonCode morton_code ) :
    MortonCode( morton_code ),
    Parent( FSVONodeAddress::InvalidAddress ),
    FirstChild( FSVONodeAddress::InvalidAddress )
{
}

int FSVOLeafNodes::GetAllocatedSize() const
{
    return LeafNodes.Num() * sizeof( FSVOLeafNode );
}

void FSVOLeafNodes::AllocateLeafNodes( const int leaf_count )
{
    LeafNodes.Reserve( leaf_count );
}

void FSVOLeafNodes::AddLeafNode( const FSVONodeAddress parent_node_address, const LeafIndex leaf_index, const SubNodeIndex sub_node_index, const bool is_occluded )
{
    if ( leaf_index > LeafNodes.Num() - 1 )
    {
        AddEmptyLeafNode( parent_node_address );
    }

    if ( is_occluded )
    {
        LeafNodes[ leaf_index ].MarkSubNodeAsOccluded( sub_node_index );
    }
}

void FSVOLeafNodes::AddEmptyLeafNode( const FSVONodeAddress parent_node_address )
{
    check( parent_node_address.LayerIndex == 1 );
    auto & leaf_node = LeafNodes.AddDefaulted_GetRef();
    leaf_node.Parent = parent_node_address;
}

FSVOLayer::FSVOLayer() :
    MaxNodeCount( -1 ),
    NodeSize( 0.0f )
{
}

FSVOLayer::FSVOLayer( const int max_node_count, const float node_size ) :
    MaxNodeCount( max_node_count ),
    NodeSize( node_size )
{
}

int FSVOLayer::GetAllocatedSize() const
{
    return Nodes.Num() * sizeof( FSVONode );
}

bool FSVOData::Initialize( const float voxel_size, const FBox & volume_bounds )
{
    Reset();

    const auto volume_size = volume_bounds.GetSize().GetAbsMax();

    const auto leaf_size = voxel_size * 4;
    const auto voxel_exponent = FMath::CeilToInt( FMath::Log2( volume_size / leaf_size ) );
    const auto layer_count = voxel_exponent + 1;

    if ( layer_count < 2 )
    {
        bIsValid = false;
        return false;
    }

    LeafNodes.Initialize( leaf_size );

    const auto navigation_bounds_size = FMath::Pow( 2, voxel_exponent ) * leaf_size;

    for ( LayerIndex layer_index = 0; layer_index < layer_count; ++layer_index )
    {
        const auto layer_edge_node_count = FMath::Pow( 2, voxel_exponent - layer_index );
        const auto layer_max_node_count = FMath::CeilToInt( FMath::Pow( layer_edge_node_count, 3 ) );
        const auto layer_voxel_size = navigation_bounds_size / layer_edge_node_count;

        Layers.Emplace( layer_max_node_count, layer_voxel_size );
    }

    NavigationBounds = FBox::BuildAABB( volume_bounds.GetCenter(), FVector( navigation_bounds_size * 0.5f ) );

    BlockedNodes.SetNumZeroed( layer_count + 1 );

    return true;
}

void FSVOData::Reset()
{
    Layers.Reset();
    LeafNodes.Reset();
}

void FSVOData::AddBlockedNode( const LayerIndex layer_index, const NodeIndex node_index )
{
    BlockedNodes[ layer_index ].Add( node_index );
}

FSVOData::FSVOData() :
    LeafNodes(),
    bIsValid( false )
{
}

int FSVOData::GetAllocatedSize() const
{
    int size = LeafNodes.GetAllocatedSize();

    for ( const auto & layer : Layers )
    {
        size += layer.GetAllocatedSize();
    }

    return size;
}
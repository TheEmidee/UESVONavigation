#pragma once

#include "SVONavigationTypes.generated.h"

class USVOPathFindingAlgorithm;
class USVOPathHeuristicCalculator;
class USVOPathTraversalCostCalculator;

typedef uint_fast64_t MortonCode;
typedef uint8 LayerIndex;
typedef uint32 NodeIndex;
typedef int32 LeafIndex;
typedef uint8 SubNodeIndex;
typedef uint8 NeighborDirection;

DECLARE_DELEGATE_ThreeParams( FSVONavigationPathQueryDelegate, uint32, ENavigationQueryResult::Type, FNavPathSharedPtr );

USTRUCT()
struct FSVODataGenerationSettings
{
    GENERATED_USTRUCT_BODY()

    FSVODataGenerationSettings()
    {
        CollisionChannel = ECollisionChannel::ECC_WorldStatic;
        Clearance = 0.0f;

        CollisionQueryParameters.bFindInitialOverlaps = true;
        CollisionQueryParameters.bTraceComplex = false;
        CollisionQueryParameters.TraceTag = "SVONavigationRasterize";
    }

    UPROPERTY( EditAnywhere, Category = "Generation" )
    TEnumAsByte< ECollisionChannel > CollisionChannel;

    UPROPERTY( EditAnywhere, Category = "Generation" )
    float Clearance;

    FCollisionQueryParams CollisionQueryParameters;
};

struct FSVONodeAddress
{
    FSVONodeAddress() :
        LayerIndex( 15 ),
        NodeIndex( 0 ),
        SubNodeIndex( 0 )
    {
    }

    explicit FSVONodeAddress( const int32 index ) :
        LayerIndex( index << 28 ),
        NodeIndex( index << 6 ),
        SubNodeIndex( index )
    {
    }

    FSVONodeAddress( const LayerIndex layer_index, const MortonCode node_index, const SubNodeIndex sub_node_index = 0 ) :
        LayerIndex( layer_index ),
        NodeIndex( node_index ),
        SubNodeIndex( sub_node_index )
    {
    }

    bool IsValid() const;
    void Invalidate();

    bool operator==( const FSVONodeAddress & other ) const
    {
        return LayerIndex == other.LayerIndex && NodeIndex == other.NodeIndex && SubNodeIndex == other.SubNodeIndex;
    }

    bool operator!=( const FSVONodeAddress & other ) const
    {
        return !operator==( other );
    }

    NavNodeRef GetNavNodeRef() const
    {
        const int32 address = LayerIndex << 28 | NodeIndex << 6 | SubNodeIndex;
        return static_cast< NavNodeRef >( address );
    }

    static const FSVONodeAddress InvalidAddress;

    uint8 LayerIndex        : 4;
    uint_fast32_t NodeIndex : 22;
    uint8 SubNodeIndex      : 6;
};

FORCEINLINE bool FSVONodeAddress::IsValid() const
{
    return LayerIndex != 15;
}

FORCEINLINE void FSVONodeAddress::Invalidate()
{
    LayerIndex = 15;
}

FORCEINLINE uint32 GetTypeHash( const FSVONodeAddress & address )
{
    return HashCombine( HashCombine( GetTypeHash( address.LayerIndex ), GetTypeHash( address.NodeIndex ) ), GetTypeHash( address.SubNodeIndex ) );
}

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVONodeAddress & data )
{
    archive.Serialize( &data, sizeof( FSVONodeAddress ) );
    return archive;
}

struct FSVOLeafNode
{
    void MarkSubNodeAsOccluded( const SubNodeIndex index );
    bool IsSubNodeOccluded( const MortonCode morton_code ) const;
    bool IsCompletelyOccluded() const;
    bool IsCompletelyFree() const;

    uint_fast64_t SubNodes = 0;
    FSVONodeAddress Parent;
};

FORCEINLINE void FSVOLeafNode::MarkSubNodeAsOccluded( const SubNodeIndex index )
{
    SubNodes |= 1ULL << index;
}

FORCEINLINE bool FSVOLeafNode::IsSubNodeOccluded( const MortonCode morton_code ) const
{
    return ( SubNodes & 1ULL << morton_code ) != 0;
}

FORCEINLINE bool FSVOLeafNode::IsCompletelyOccluded() const
{
    return SubNodes == -1;
}

FORCEINLINE bool FSVOLeafNode::IsCompletelyFree() const
{
    return SubNodes == 0;
}

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVOLeafNode & data )
{
    archive << data.SubNodes;
    archive << data.Parent;
    return archive;
}

struct FSVONode
{
    FSVONode();
    explicit FSVONode( MortonCode morton_code );
    bool HasChildren() const;

    MortonCode MortonCode;
    FSVONodeAddress Parent;
    FSVONodeAddress FirstChild;
    FSVONodeAddress Neighbors[ 6 ];
};

FORCEINLINE bool FSVONode::HasChildren() const
{
    return FirstChild.IsValid();
}

FORCEINLINE bool operator<( const FSVONode & left, const FSVONode & right )
{
    return left.MortonCode < right.MortonCode;
}

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVONode & data )
{
    archive << data.MortonCode;
    archive << data.Parent;
    archive << data.FirstChild;

    for ( int32 neighbor_index = 0; neighbor_index < 6; neighbor_index++ )
    {
        archive << data.Neighbors[ neighbor_index ];
    }

    return archive;
}

class FSVOLeafNodes
{
public:
    friend FArchive & operator<<( FArchive & archive, FSVOLeafNodes & leaf_nodes );
    friend class FSVOVolumeNavigationData;
    friend class FSVOData;

    const FSVOLeafNode & GetLeafNode( const LeafIndex leaf_index ) const;
    const TArray< FSVOLeafNode > & GetLeafNodes() const;
    float GetLeafNodeSize() const;
    float GetLeafNodeExtent() const;
    float GetLeafSubNodeSize() const;
    float GetLeafSubNodeExtent() const;

    int GetAllocatedSize() const;

private:
    FSVOLeafNode & GetLeafNode( const LeafIndex leaf_index );

    void Initialize( float leaf_size );
    void Reset();
    void AllocateLeafNodes( int leaf_count );
    void AddLeafNode( LeafIndex leaf_index, SubNodeIndex sub_node_index, bool is_occluded );
    void AddEmptyLeafNode();

    float LeafNodeSize;
    TArray< FSVOLeafNode > LeafNodes;
};

FORCEINLINE const FSVOLeafNode & FSVOLeafNodes::GetLeafNode( const LeafIndex leaf_index ) const
{
    return LeafNodes[ leaf_index ];
}

FORCEINLINE const TArray< FSVOLeafNode > & FSVOLeafNodes::GetLeafNodes() const
{
    return LeafNodes;
}

FORCEINLINE float FSVOLeafNodes::GetLeafNodeSize() const
{
    return LeafNodeSize;
}

FORCEINLINE float FSVOLeafNodes::GetLeafNodeExtent() const
{
    return GetLeafNodeSize() * 0.5f;
}

FORCEINLINE float FSVOLeafNodes::GetLeafSubNodeSize() const
{
    return GetLeafNodeSize() * 0.25f;
}

FORCEINLINE float FSVOLeafNodes::GetLeafSubNodeExtent() const
{
    return GetLeafSubNodeSize() * 0.5f;
}

FORCEINLINE FSVOLeafNode & FSVOLeafNodes::GetLeafNode( const LeafIndex leaf_index )
{
    return LeafNodes[ leaf_index ];
}

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVOLeafNodes & leaf_nodes )
{
    archive << leaf_nodes.LeafNodes;
    archive << leaf_nodes.LeafNodeSize;
    return archive;
}

class FSVOLayer
{
public:
    friend FArchive & operator<<( FArchive & archive, FSVOLayer & layer );
    friend class FSVOVolumeNavigationData;

    FSVOLayer();
    FSVOLayer( int max_node_count, float node_size );

    const TArray< FSVONode > & GetNodes() const;
    int32 GetNodeCount() const;
    const FSVONode & GetNode( NodeIndex node_index ) const;
    float GetNodeSize() const;
    float GetNodeExtent() const;
    uint32 GetMaxNodeCount() const;

    int GetAllocatedSize() const;

private:
    TArray< FSVONode > & GetNodes();

    TArray< FSVONode > Nodes;
    int MaxNodeCount;
    float NodeSize;
};

FORCEINLINE const TArray< FSVONode > & FSVOLayer::GetNodes() const
{
    return Nodes;
}

FORCEINLINE TArray< FSVONode > & FSVOLayer::GetNodes()
{
    return Nodes;
}

FORCEINLINE int32 FSVOLayer::GetNodeCount() const
{
    return Nodes.Num();
}

FORCEINLINE const FSVONode & FSVOLayer::GetNode( const NodeIndex node_index ) const
{
    return Nodes[ node_index ];
}

FORCEINLINE float FSVOLayer::GetNodeSize() const
{
    return NodeSize;
}

FORCEINLINE float FSVOLayer::GetNodeExtent() const
{
    return GetNodeSize() * 0.5f;
}

FORCEINLINE uint32 FSVOLayer::GetMaxNodeCount() const
{
    return MaxNodeCount;
}

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVOLayer & layer )
{
    archive << layer.Nodes;
    archive << layer.NodeSize;
    return archive;
}

class FSVOData
{
public:
    friend FArchive & operator<<( FArchive & archive, FSVOData & data );
    friend class FSVOVolumeNavigationData;

    FSVOData();

    int GetLayerCount() const;
    const FSVOLayer & GetLayer( LayerIndex layer_index ) const;
    const FSVOLayer & GetLastLayer() const;
    const FSVOLeafNodes & GetLeafNodes() const;
    const FBox & GetNavigationBounds() const;
    bool IsValid() const;

    int GetAllocatedSize() const;

private:
    FSVOLayer & GetLayer( LayerIndex layer_index );
    FSVOLeafNodes & GetLeafNodes();
    bool Initialize( float voxel_size, const FBox & volume_bounds );
    void Reset();
    void AddBlockedNode( LayerIndex layer_index, NodeIndex node_index );
    const TArray< NodeIndex > & GetLayerBlockedNodes( LayerIndex layer_index ) const;

    TArray< TArray< NodeIndex > > BlockedNodes;
    TArray< FSVOLayer > Layers;
    FSVOLeafNodes LeafNodes;
    FBox NavigationBounds;
    uint8 bIsValid : 1;
};

FORCEINLINE int FSVOData::GetLayerCount() const
{
    return Layers.Num();
}

FORCEINLINE FSVOLayer & FSVOData::GetLayer( const LayerIndex layer_index )
{
    return Layers[ layer_index ];
}

FORCEINLINE const FSVOLayer & FSVOData::GetLayer( const LayerIndex layer_index ) const
{
    return Layers[ layer_index ];
}

FORCEINLINE const FSVOLayer & FSVOData::GetLastLayer() const
{
    return Layers.Last();
}

FORCEINLINE const FSVOLeafNodes & FSVOData::GetLeafNodes() const
{
    return LeafNodes;
}

FORCEINLINE FSVOLeafNodes & FSVOData::GetLeafNodes()
{
    return LeafNodes;
}

FORCEINLINE const FBox & FSVOData::GetNavigationBounds() const
{
    return NavigationBounds;
}

FORCEINLINE bool FSVOData::IsValid() const
{
    return bIsValid && GetLayerCount() > 0;
}

FORCEINLINE const TArray< NodeIndex > & FSVOData::GetLayerBlockedNodes( const LayerIndex layer_index ) const
{
    return BlockedNodes[ layer_index ];
}

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVOData & data )
{
    archive << data.Layers;
    archive << data.LeafNodes;
    archive << data.NavigationBounds;

    if ( archive.IsLoading() )
    {
        data.bIsValid = ( data.Layers.Num() > 0 && data.NavigationBounds.IsValid );

        if ( !data.bIsValid )
        {
            data.Reset();
        }
    }

    return archive;
}
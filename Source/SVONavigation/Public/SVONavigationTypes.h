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

struct FSVOPathFindingResult
{
    FNavPathSharedPtr Path;
    ENavigationQueryResult::Type Result;

    explicit FSVOPathFindingResult( ENavigationQueryResult::Type result = ENavigationQueryResult::Invalid ) :
        Result( result )
    {
    }

    bool IsSuccessful() const;
};

FORCEINLINE bool FSVOPathFindingResult::IsSuccessful() const
{
    return Result == ENavigationQueryResult::Success;
}

USTRUCT()
struct SVONAVIGATION_API FSVOVolumeNavigationDataDebugInfos
{
    GENERATED_USTRUCT_BODY()

    FSVOVolumeNavigationDataDebugInfos() :
        bDebugDrawBounds( false ),
        bDebugDrawLayers( false ),
        LayerIndexToDraw( 1 ),
        bDebugDrawSubNodes( false ),
        DebugDrawOccludedVoxels( true ),
        DebugDrawFreeVoxels( false )
    {
    }

    UPROPERTY( EditInstanceOnly )
    uint8 bDebugDrawBounds : 1;

    UPROPERTY( EditInstanceOnly )
    uint8 bDebugDrawLayers : 1;

    UPROPERTY( EditInstanceOnly, meta = ( EditCondition = "bDebugDrawLayers", ClampMin = "0", UIMin = "0" ) )
    uint8 LayerIndexToDraw;

    UPROPERTY( EditInstanceOnly )
    uint8 bDebugDrawSubNodes : 1;

    UPROPERTY( EditInstanceOnly )
    uint8 DebugDrawOccludedVoxels : 1;

    UPROPERTY( EditInstanceOnly )
    uint8 DebugDrawFreeVoxels : 1;
};

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

struct FSVOLeaf
{
    void MarkSubNodeAsOccluded( const SubNodeIndex index );
    bool IsSubNodeOccluded( const MortonCode morton_code ) const;
    bool IsCompletelyOccluded() const;
    bool IsCompletelyFree() const;

    uint_fast64_t SubNodes = 0;
};

FORCEINLINE void FSVOLeaf::MarkSubNodeAsOccluded( const SubNodeIndex index )
{
    SubNodes |= 1ULL << index;
}

FORCEINLINE bool FSVOLeaf::IsSubNodeOccluded( const MortonCode morton_code ) const
{
    return ( SubNodes & 1ULL << morton_code ) != 0;
}

FORCEINLINE bool FSVOLeaf::IsCompletelyOccluded() const
{
    return SubNodes == -1;
}

FORCEINLINE bool FSVOLeaf::IsCompletelyFree() const
{
    return SubNodes == 0;
}

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVOLeaf & data )
{
    archive << data.SubNodes;
    return archive;
}

struct FSVOOctreeLink
{
    FSVOOctreeLink() :
        LayerIndex( 15 ),
        NodeIndex( 0 ),
        SubNodeIndex( 0 )
    {
    }

    explicit FSVOOctreeLink( const int32 index ) :
        LayerIndex( index << 28 ),
        NodeIndex( index << 6 ),
        SubNodeIndex( index )
    {
    }

    FSVOOctreeLink( const LayerIndex layer_index, const MortonCode node_index, const SubNodeIndex sub_node_index ) :
        LayerIndex( layer_index ),
        NodeIndex( node_index ),
        SubNodeIndex( sub_node_index )
    {
    }

    bool IsValid() const;
    void Invalidate();

    bool operator==( const FSVOOctreeLink & other ) const
    {
        return LayerIndex == other.LayerIndex && NodeIndex == other.NodeIndex && SubNodeIndex == other.SubNodeIndex;
    }

    bool operator!=( const FSVOOctreeLink & other ) const
    {
        return !operator==( other );
    }

    NavNodeRef GetNavNodeRef() const
    {
        const int32 link = LayerIndex << 28 | NodeIndex << 6 | SubNodeIndex;
        return static_cast< NavNodeRef >( link );
    }

    static FSVOOctreeLink InvalidLink()
    {
        return FSVOOctreeLink();
    }

    uint8 LayerIndex : 4;
    uint_fast32_t NodeIndex : 22;
    uint8 SubNodeIndex : 6;
};

FORCEINLINE bool FSVOOctreeLink::IsValid() const
{
    return LayerIndex != 15;
}

FORCEINLINE void FSVOOctreeLink::Invalidate()
{
    LayerIndex = 15;
}

FORCEINLINE uint32 GetTypeHash( const FSVOOctreeLink & link )
{
    return HashCombine( HashCombine( GetTypeHash( link.LayerIndex ), GetTypeHash( link.NodeIndex ) ), GetTypeHash( link.SubNodeIndex ) );
}

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVOOctreeLink & data )
{
    archive.Serialize( &data, sizeof( FSVOOctreeLink ) );
    return archive;
}

struct FSVOOctreeNode
{
    MortonCode MortonCode;
    FSVOOctreeLink Parent;
    FSVOOctreeLink FirstChild;
    FSVOOctreeLink Neighbors[ 6 ];

    FSVOOctreeNode() :
        MortonCode( 0 ),
        Parent( FSVOOctreeLink::InvalidLink() ),
        FirstChild( FSVOOctreeLink::InvalidLink() )
    {
    }

    bool HasChildren() const
    {
        return FirstChild.IsValid();
    }
};

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVOOctreeNode & data )
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

class FSVOLeaves
{
public:
    friend FArchive & operator<<( FArchive & archive, FSVOLeaves & leaves );
    friend class FSVOVolumeNavigationData;
    friend class FSVOOctreeData;

    const FSVOLeaf & GetLeaf( const LeafIndex leaf_index ) const;
    const TArray< FSVOLeaf > & GetLeaves() const;
    float GetLeafExtent() const;
    float GetLeafHalfExtent() const;
    float GetLeafSubNodeExtent() const;
    float GetLeafSubNodeHalfExtent() const;

    int GetAllocatedSize() const;

private:
    FSVOLeaf GetLeaf( const LeafIndex leaf_index );

    void Initialize( float leaf_extent );
    void Reset();
    void AllocateLeaves( int leaf_count );
    void AddLeaf( LeafIndex leaf_index, SubNodeIndex subnode_index, bool is_occluded );
    void AddEmptyLeaf();

    float LeafExtent;
    TArray< FSVOLeaf > Leaves;
};

FORCEINLINE const FSVOLeaf & FSVOLeaves::GetLeaf( const LeafIndex leaf_index ) const
{
    return Leaves[ leaf_index ];
}

FORCEINLINE const TArray< FSVOLeaf > & FSVOLeaves::GetLeaves() const
{
    return Leaves;
}

FORCEINLINE float FSVOLeaves::GetLeafExtent() const
{
    return LeafExtent;
}

FORCEINLINE float FSVOLeaves::GetLeafHalfExtent() const
{
    return GetLeafExtent() * 0.5f;
}

FORCEINLINE float FSVOLeaves::GetLeafSubNodeExtent() const
{
    return GetLeafExtent() * 0.25f;
}

FORCEINLINE float FSVOLeaves::GetLeafSubNodeHalfExtent() const
{
    return GetLeafSubNodeExtent() * 0.5f;
}

FORCEINLINE FSVOLeaf FSVOLeaves::GetLeaf( const LeafIndex leaf_index )
{
    return Leaves[ leaf_index ];
}

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVOLeaves & leaves )
{
    archive << leaves.Leaves;
    archive << leaves.LeafExtent;
    return archive;
}

class FSVOLayer
{
public:
    friend FArchive & operator<<( FArchive & archive, FSVOLayer & layer );
    friend class FSVOVolumeNavigationData;

    FSVOLayer();
    FSVOLayer( int max_node_count, float voxel_extent );

    const TArray< FSVOOctreeNode > & GetNodes() const;
    int32 GetNodeCount() const;
    const FSVOOctreeNode & GetNode( NodeIndex node_index ) const;
    float GetVoxelExtent() const;
    float GetVoxelHalfExtent() const;
    uint32 GetMaxNodeCount() const;
    uint32 GetBlockedNodesCount() const;
    const TSet< MortonCode > & GetBlockedNodes() const;

    int GetAllocatedSize() const;

private:
    TArray< FSVOOctreeNode > & GetNodes();
    void AddBlockedNode( NodeIndex node_index );

    TArray< FSVOOctreeNode > Nodes;
    TSet< MortonCode > BlockedNodes;
    int MaxNodeCount;
    float VoxelExtent;
};

FORCEINLINE const TArray< FSVOOctreeNode > & FSVOLayer::GetNodes() const
{
    return Nodes;
}

FORCEINLINE TArray< FSVOOctreeNode > & FSVOLayer::GetNodes()
{
    return Nodes;
}

FORCEINLINE int32 FSVOLayer::GetNodeCount() const
{
    return Nodes.Num();
}

FORCEINLINE const FSVOOctreeNode & FSVOLayer::GetNode( const NodeIndex node_index ) const
{
    return Nodes[ node_index ];
}

FORCEINLINE float FSVOLayer::GetVoxelExtent() const
{
    return VoxelExtent;
}

FORCEINLINE float FSVOLayer::GetVoxelHalfExtent() const
{
    return GetVoxelExtent() * 0.5f;
}

FORCEINLINE uint32 FSVOLayer::GetMaxNodeCount() const
{
    return MaxNodeCount;
}

FORCEINLINE const TSet< MortonCode > & FSVOLayer::GetBlockedNodes() const
{
    return BlockedNodes;
}

FORCEINLINE uint32 FSVOLayer::GetBlockedNodesCount() const
{
    return BlockedNodes.Num();
}

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVOLayer & layer )
{
    archive << layer.Nodes;
    archive << layer.VoxelExtent;
    return archive;
}

class FSVOOctreeData
{
public:
    friend FArchive & operator<<( FArchive & archive, FSVOOctreeData & data );
    friend class FSVOVolumeNavigationData;

    FSVOOctreeData();

    int GetLayerCount() const;
    const FSVOLayer & GetLayer( LayerIndex layer_index ) const;
    const FSVOLayer & GetLastLayer() const;
    const FSVOLeaves & GetLeaves() const;
    const FBox & GetNavigationBounds() const;
    bool IsValid() const;

    int GetAllocatedSize() const;

private:
    FSVOLayer & GetLayer( LayerIndex layer_index );
    FSVOLeaves & GetLeaves();
    bool Initialize( float voxel_extent, const FBox & volume_bounds );
    void Reset();

    TArray< FSVOLayer > Layers;
    FSVOLeaves Leaves;
    FBox NavigationBounds;
    uint8 bIsValid : 1;
};

FORCEINLINE int FSVOOctreeData::GetLayerCount() const
{
    return Layers.Num();
}

FORCEINLINE FSVOLayer & FSVOOctreeData::GetLayer( const LayerIndex layer_index )
{
    return Layers[ layer_index ];
}

FORCEINLINE const FSVOLayer & FSVOOctreeData::GetLayer( const LayerIndex layer_index ) const
{
    return Layers[ layer_index ];
}

FORCEINLINE const FSVOLayer & FSVOOctreeData::GetLastLayer() const
{
    return Layers.Last();
}

FORCEINLINE const FSVOLeaves & FSVOOctreeData::GetLeaves() const
{
    return Leaves;
}

FORCEINLINE FSVOLeaves & FSVOOctreeData::GetLeaves()
{
    return Leaves;
}

FORCEINLINE const FBox & FSVOOctreeData::GetNavigationBounds() const
{
    return NavigationBounds;
}

FORCEINLINE bool FSVOOctreeData::IsValid() const
{
    return bIsValid && GetLayerCount() > 0;
}

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVOOctreeData & data )
{
    archive << data.Layers;
    archive << data.Leaves;
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

USTRUCT()
struct SVONAVIGATION_API FSVONavigationQueryFilterSettings
{
    GENERATED_USTRUCT_BODY()

    FSVONavigationQueryFilterSettings();

    UPROPERTY( EditAnywhere, Instanced )
    USVOPathFindingAlgorithm * PathFinder;

    UPROPERTY( EditAnywhere, Instanced )
    USVOPathTraversalCostCalculator * TraversalCostCalculator;

    UPROPERTY( EditAnywhere, Instanced )
    USVOPathHeuristicCalculator * HeuristicCalculator;

    UPROPERTY( EditDefaultsOnly )
    float HeuristicScale;

    // If set to true, this will lower the cost of traversing bigger nodes, and make the pathfinding more favorable traversing them
    UPROPERTY( EditDefaultsOnly )
    uint8 bUseNodeSizeCompensation : 1;

    UPROPERTY( EditDefaultsOnly )
    uint8 bOffsetPathVerticallyByAgentRadius : 1;
};

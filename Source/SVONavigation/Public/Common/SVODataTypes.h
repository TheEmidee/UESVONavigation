#pragma once

#include "SVONodeTypes.h"

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
    const FBox & GetVolumeBounds() const;
    bool IsValid() const;

    void Reset();
    int GetAllocatedSize() const;

private:
    FSVOLayer & GetLayer( LayerIndex layer_index );
    FSVOLeafNodes & GetLeafNodes();
    bool Initialize( float voxel_size, const FBox & volume_bounds );
    void AddBlockedNode( LayerIndex layer_index, NodeIndex node_index );
    const TArray< NodeIndex > & GetLayerBlockedNodes( LayerIndex layer_index ) const;

    TArray< TArray< NodeIndex > > BlockedNodes;
    TArray< FSVOLayer > Layers;
    FSVOLeafNodes LeafNodes;
    FBox NavigationBounds;
    // The bounds of the nav mesh bounds volume in the world
    FBox VolumeBounds;
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

FORCEINLINE const FBox & FSVOData::GetVolumeBounds() const
{
    return VolumeBounds;
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
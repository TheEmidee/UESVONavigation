#pragma once
#include "SVONavigationTypes.h"

struct FSVOBoundsNavigationDataGenerationSettings
{
    float VoxelSize;
    UWorld * World;
    FSVODataGenerationSettings GenerationSettings;
};

class SVONAVIGATION_API FSVOBoundsNavigationData
{
public:
    friend FArchive & operator<<( FArchive & archive, FSVOBoundsNavigationData & data );

    const FBox & GetVolumeBounds() const;
    const FBox & GetNavigationBounds() const;
    const FSVOOctreeData & GetOctreeData() const;
    FVector GetNodePosition( LayerIndex layer_index, MortonCode morton_code ) const;
    FVector GetNodePositionFromLink( const FSVOOctreeLink & link ) const;
    float GetLayerVoxelSize( LayerIndex layer_index ) const;
    float GetLayerVoxelHalfSize( LayerIndex layer_index ) const;
    uint32 GetLayerNodeCount( LayerIndex layer_index ) const;
    const TArray< FSVOOctreeNode > & GetLayerNodes( LayerIndex layer_index ) const;
    const FSVOOctreeNode & GetNodeFromLink( const FSVOOctreeLink & link ) const;
    const FSVOOctreeLeaf & GetLeafNode( LeafIndex leaf_index ) const;
    TOptional< FSVOOctreeLink > GetLinkFromPosition( const FVector & position ) const;
    bool GetLinkPosition( FVector & position, const FSVOOctreeLink & link ) const;
    void GetLeafNeighbors( TArray< FSVOOctreeLink > & neighbors, const FSVOOctreeLink & link ) const;
    void GetNeighbors( TArray< FSVOOctreeLink > & neighbors, const FSVOOctreeLink & link ) const;

    void GenerateNavigationData( const FBox & volume_bounds, const FSVOBoundsNavigationDataGenerationSettings & generation_settings );

private:
    TArray< FSVOOctreeNode > & GetLayerNodes( LayerIndex layer_index );
    int32 GetLayerMaxNodeCount( LayerIndex layer_index ) const;
    bool IsPositionOccluded( const FVector & position, float box_size ) const;
    void FirstPassRasterization();
    void AllocateLeafNodes();
    void RasterizeLeaf( const FVector & node_position, int32 leaf_index );
    void RasterizeInitialLayer();
    void RasterizeLayer( LayerIndex layer_index );
    TOptional< NodeIndex > GetNodeIndexFromMortonCode( LayerIndex layer_index, MortonCode morton_code ) const;
    void BuildNeighborLinks( LayerIndex layer_index );
    bool FindNeighborInDirection( FSVOOctreeLink & link, const LayerIndex layer_index, const NodeIndex node_index, const NeighborDirection direction, const FVector & node_position );

    FSVOBoundsNavigationDataGenerationSettings Settings;
    int VoxelExponent;
    uint8 LayerCount = 0;
    FBox NavigationBounds;
    FBox VolumeBounds;
    float UsedBoxExtent;
    FSVOOctreeData SVOData;
    TArray< TSet< MortonCode > > BlockedIndices;
    TArray< float > LayerVoxelSizes;
    TArray< float > LayerVoxelHalfSizes;
    TArray< uint32 > LayerNodeCount;
};

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVOBoundsNavigationData & data )
{
    archive << data.NavigationBounds;
    archive << data.VolumeBounds;
    archive << data.VoxelExponent;
    archive << data.LayerCount;
    archive << data.UsedBoxExtent;
    archive << data.LayerVoxelSizes;
    archive << data.LayerVoxelHalfSizes;
    archive << data.LayerNodeCount;
    archive << data.SVOData;

    archive << data.VolumeBounds;
    return archive;
}

FORCEINLINE const FBox & FSVOBoundsNavigationData::GetVolumeBounds() const
{
    return VolumeBounds;
}

FORCEINLINE const FBox & FSVOBoundsNavigationData::GetNavigationBounds() const
{
    return NavigationBounds;
}

FORCEINLINE const FSVOOctreeData & FSVOBoundsNavigationData::GetOctreeData() const
{
    return SVOData;
}

FORCEINLINE float FSVOBoundsNavigationData::GetLayerVoxelSize( LayerIndex layer_index ) const
{
    return LayerVoxelSizes[ layer_index ];
}

FORCEINLINE float FSVOBoundsNavigationData::GetLayerVoxelHalfSize( LayerIndex layer_index ) const
{
    return LayerVoxelHalfSizes[ layer_index ];
}

FORCEINLINE uint32 FSVOBoundsNavigationData::GetLayerNodeCount( LayerIndex layer_index ) const
{
    return LayerNodeCount[ layer_index ];
}

FORCEINLINE const TArray< FSVOOctreeNode > & FSVOBoundsNavigationData::GetLayerNodes( LayerIndex layer_index ) const
{
    return SVOData.NodesByLayers[ layer_index ];
}

FORCEINLINE TArray< FSVOOctreeNode > & FSVOBoundsNavigationData::GetLayerNodes( LayerIndex layer_index )
{
    return SVOData.NodesByLayers[ layer_index ];
}

FORCEINLINE int32 FSVOBoundsNavigationData::GetLayerMaxNodeCount( LayerIndex layer_index ) const
{
    return FMath::Pow( 2, VoxelExponent - layer_index );
}

FORCEINLINE const FSVOOctreeNode & FSVOBoundsNavigationData::GetNodeFromLink( const FSVOOctreeLink & link ) const
{
    return SVOData.NodesByLayers[ link.LayerIndex ][ link.NodeIndex ];
}

FORCEINLINE const FSVOOctreeLeaf & FSVOBoundsNavigationData::GetLeafNode( LeafIndex leaf_index ) const
{
    return SVOData.Leaves[ leaf_index ];
}
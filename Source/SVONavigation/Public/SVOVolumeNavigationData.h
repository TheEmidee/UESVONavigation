#pragma once

#include "SVONavigationTypes.h"

enum class ESVOVersion : uint8;

struct FSVOVolumeNavigationDataGenerationSettings
{
    FSVOVolumeNavigationDataGenerationSettings();

    float VoxelExtent;
    UWorld * World;
    FSVODataGenerationSettings GenerationSettings;
};

class SVONAVIGATION_API FSVOVolumeNavigationData
{
public:
    typedef FSVOOctreeLink FNodeRef;

    FSVOVolumeNavigationData() = default;

    // Used by FGraphAStar
    bool IsValidRef( const FSVOOctreeLink ref ) const
    {
        return ref.IsValid();
    }

    const FSVOVolumeNavigationDataGenerationSettings & GetDataGenerationSettings() const;
    const FBox & GetVolumeBounds() const;
    const FSVOOctreeData & GetOctreeData() const;
    FVector GetNodePosition( const LayerIndex layer_index, MortonCode morton_code ) const;
    FVector GetLinkPosition( const FSVOOctreeLink & link ) const;
    const FSVOOctreeNode & GetNodeFromLink( const FSVOOctreeLink & link ) const;
    bool GetLinkFromPosition( FSVOOctreeLink & link, const FVector & position ) const;
    void GetNeighbors( TArray< FSVOOctreeLink > & neighbors, const FSVOOctreeLink & link ) const;
    float GetLayerRatio( LayerIndex layer_index ) const;
    float GetLayerInverseRatio( LayerIndex layer_index ) const;
    float GetVoxelHalfExtentFromLink( FSVOOctreeLink link ) const;
    TOptional< FNavLocation > GetRandomPoint() const;

    void GenerateNavigationData( const FBox & volume_bounds, const FSVOVolumeNavigationDataGenerationSettings & generation_settings );
    void Serialize( FArchive & archive, const ESVOVersion version );

private:
    int GetLayerCount() const;
    bool IsPositionOccluded( const FVector & position, float box_half_extent ) const;
    void FirstPassRasterization();
    void RasterizeLeaf( const FVector & node_position, LeafIndex leaf_index );
    void RasterizeInitialLayer();
    void RasterizeLayer( LayerIndex layer_index );
    TOptional< NodeIndex > GetNodeIndexFromMortonCode( LayerIndex layer_index, MortonCode morton_code ) const;
    void BuildNeighborLinks( LayerIndex layer_index );
    bool FindNeighborInDirection( FSVOOctreeLink & link, const LayerIndex layer_index, const NodeIndex node_index, const NeighborDirection direction, const FVector & node_position );
    void GetLeafNeighbors( TArray< FSVOOctreeLink > & neighbors, const FSVOOctreeLink & link ) const;
    void GetFreeNodesFromLink( FSVOOctreeLink link, TArray< FSVOOctreeLink > & free_nodes ) const;

    FSVOVolumeNavigationDataGenerationSettings Settings;
    FBox VolumeBounds;
    FSVOOctreeData SVOData;
};

FORCEINLINE const FSVOVolumeNavigationDataGenerationSettings & FSVOVolumeNavigationData::GetDataGenerationSettings() const
{
    return Settings;
}

FORCEINLINE const FBox & FSVOVolumeNavigationData::GetVolumeBounds() const
{
    return VolumeBounds;
}

FORCEINLINE const FSVOOctreeData & FSVOVolumeNavigationData::GetOctreeData() const
{
    return SVOData;
}

FORCEINLINE const FSVOOctreeNode & FSVOVolumeNavigationData::GetNodeFromLink( const FSVOOctreeLink & link ) const
{
    return link.LayerIndex < 15
               ? SVOData.GetLayer( link.LayerIndex ).GetNode( link.NodeIndex )
               : SVOData.GetLastLayer().GetNode( 0 );
}

FORCEINLINE int FSVOVolumeNavigationData::GetLayerCount() const
{
    return SVOData.GetLayerCount();
}
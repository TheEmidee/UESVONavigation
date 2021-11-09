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
    typedef FSVONodeAddress FNodeRef;

    FSVOVolumeNavigationData() = default;

    // Used by FGraphAStar
    bool IsValidRef( const FSVONodeAddress ref ) const
    {
        return ref.IsValid();
    }

    const FSVOVolumeNavigationDataGenerationSettings & GetDataGenerationSettings() const;
    const FBox & GetVolumeBounds() const;
    const FSVOData & GetOctreeData() const;
    FVector GetNodePosition( const LayerIndex layer_index, MortonCode morton_code ) const;
    FVector GetLinkPosition( const FSVONodeAddress & link ) const;
    const FSVONode & GetNodeFromLink( const FSVONodeAddress & link ) const;
    bool GetLinkFromPosition( FSVONodeAddress & link, const FVector & position ) const;
    void GetNeighbors( TArray< FSVONodeAddress > & neighbors, const FSVONodeAddress & link ) const;
    float GetLayerRatio( LayerIndex layer_index ) const;
    float GetLayerInverseRatio( LayerIndex layer_index ) const;
    float GetVoxelHalfExtentFromLink( FSVONodeAddress link ) const;
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
    bool FindNeighborInDirection( FSVONodeAddress & link, const LayerIndex layer_index, const NodeIndex node_index, const NeighborDirection direction, const FVector & node_position );
    void GetLeafNeighbors( TArray< FSVONodeAddress > & neighbors, const FSVONodeAddress & link ) const;
    void GetFreeNodesFromLink( FSVONodeAddress link, TArray< FSVONodeAddress > & free_nodes ) const;

    FSVOVolumeNavigationDataGenerationSettings Settings;
    FBox VolumeBounds;
    FSVOData SVOData;
};

FORCEINLINE const FSVOVolumeNavigationDataGenerationSettings & FSVOVolumeNavigationData::GetDataGenerationSettings() const
{
    return Settings;
}

FORCEINLINE const FBox & FSVOVolumeNavigationData::GetVolumeBounds() const
{
    return VolumeBounds;
}

FORCEINLINE const FSVOData & FSVOVolumeNavigationData::GetOctreeData() const
{
    return SVOData;
}

FORCEINLINE const FSVONode & FSVOVolumeNavigationData::GetNodeFromLink( const FSVONodeAddress & link ) const
{
    return link.LayerIndex < 15
               ? SVOData.GetLayer( link.LayerIndex ).GetNode( link.NodeIndex )
               : SVOData.GetLastLayer().GetNode( 0 );
}

FORCEINLINE int FSVOVolumeNavigationData::GetLayerCount() const
{
    return SVOData.GetLayerCount();
}
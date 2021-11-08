#pragma once

#include "SVONavigationTypes.h"

enum class ESVOVersion : uint8;

struct FSVOBoundsNavigationDataGenerationSettings
{
    FSVOBoundsNavigationDataGenerationSettings();

    float VoxelExtent;
    UWorld * World;
    FSVODataGenerationSettings GenerationSettings;
};

class SVONAVIGATION_API FSVOBoundsNavigationData
{
public:
    typedef FSVOOctreeLink FNodeRef;

    FSVOBoundsNavigationData() = default;

    // Used by FGraphAStar
    bool IsValidRef( const FSVOOctreeLink ref ) const
    {
        return ref.IsValid();
    }

    const FSVOBoundsNavigationDataGenerationSettings & GetDataGenerationSettings() const;
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

    void GenerateNavigationData( const FBox & volume_bounds, const FSVOBoundsNavigationDataGenerationSettings & generation_settings );
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

    FSVOBoundsNavigationDataGenerationSettings Settings;
    FBox VolumeBounds;
    FSVOOctreeData SVOData;
};

FORCEINLINE const FSVOBoundsNavigationDataGenerationSettings & FSVOBoundsNavigationData::GetDataGenerationSettings() const
{
    return Settings;
}

FORCEINLINE const FBox & FSVOBoundsNavigationData::GetVolumeBounds() const
{
    return VolumeBounds;
}

FORCEINLINE const FSVOOctreeData & FSVOBoundsNavigationData::GetOctreeData() const
{
    return SVOData;
}

FORCEINLINE const FSVOOctreeNode & FSVOBoundsNavigationData::GetNodeFromLink( const FSVOOctreeLink & link ) const
{
    return link.LayerIndex < 15
               ? SVOData.GetLayer( link.LayerIndex ).GetNode( link.NodeIndex )
               : SVOData.GetLastLayer().GetNode( 0 );
}

FORCEINLINE int FSVOBoundsNavigationData::GetLayerCount() const
{
    return SVOData.GetLayerCount();
}
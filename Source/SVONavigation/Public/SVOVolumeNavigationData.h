#pragma once

#include "SVONavigationTypes.h"

#include <Templates/SubclassOf.h>

class UNavigationQueryFilter;
class USVONavigationQueryFilter;
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
    const FBox & GetNavigationBounds() const;
    const FSVOData & GetData() const;
    const FSVONode & GetNodeFromAddress( const FSVONodeAddress & address ) const;
    TSubclassOf< USVONavigationQueryFilter > GetVolumeNavigationQueryFilter() const;
    void SetVolumeNavigationQueryFilter( TSubclassOf< USVONavigationQueryFilter > navigation_query_filter );

    FVector GetNodePositionFromAddress( const FSVONodeAddress & address, bool try_get_sub_node_position ) const;
    FVector GetLeafNodePositionFromMortonCode( MortonCode morton_code ) const;
    bool GetNodeAddressFromPosition( FSVONodeAddress & node_address, const FVector & position ) const;
    void GetNodeNeighbors( TArray< FSVONodeAddress > & neighbors, const FSVONodeAddress & node_address ) const;
    float GetLayerRatio( LayerIndex layer_index ) const;
    float GetLayerInverseRatio( LayerIndex layer_index ) const;
    float GetNodeExtentFromNodeAddress( FSVONodeAddress node_address ) const;
    TOptional< FNavLocation > GetRandomPoint() const;

    void GenerateNavigationData( const FBox & volume_bounds, const FSVOVolumeNavigationDataGenerationSettings & generation_settings );
    void Serialize( FArchive & archive, const ESVOVersion version );

private:
    int GetLayerCount() const;
    bool IsPositionOccluded( const FVector & position, float box_extent ) const;
    void FirstPassRasterization();
    void RasterizeLeaf( const FVector & node_position, const LeafIndex leaf_index );
    void RasterizeInitialLayer( TMap< LeafIndex, MortonCode > & leaf_index_to_layer_one_node_index_map );
    void RasterizeLayer( LayerIndex layer_index );
    int32 GetNodeIndexFromMortonCode( LayerIndex layer_index, MortonCode morton_code ) const;
    void BuildNeighborLinks( LayerIndex layer_index );
    bool FindNeighborInDirection( FSVONodeAddress & node_address, const LayerIndex layer_index, const NodeIndex node_index, const NeighborDirection direction );
    void GetLeafNeighbors( TArray< FSVONodeAddress > & neighbors, const FSVONodeAddress & leaf_address ) const;
    void GetFreeNodesFromNodeAddress( FSVONodeAddress node_address, TArray< FSVONodeAddress > & free_nodes ) const;
    void BuildParentLinkForLeafNodes( const TMap< LeafIndex, MortonCode > & leaf_index_to_parent_morton_code_map );

    FSVOVolumeNavigationDataGenerationSettings Settings;
    FBox VolumeBounds;
    FSVOData SVOData;
    TSubclassOf< USVONavigationQueryFilter > VolumeNavigationQueryFilter;
};

FORCEINLINE const FSVOVolumeNavigationDataGenerationSettings & FSVOVolumeNavigationData::GetDataGenerationSettings() const
{
    return Settings;
}

FORCEINLINE const FBox & FSVOVolumeNavigationData::GetVolumeBounds() const
{
    return VolumeBounds;
}

FORCEINLINE const FBox & FSVOVolumeNavigationData::GetNavigationBounds() const
{
    return SVOData.GetNavigationBounds();
}

FORCEINLINE const FSVOData & FSVOVolumeNavigationData::GetData() const
{
    return SVOData;
}

FORCEINLINE const FSVONode & FSVOVolumeNavigationData::GetNodeFromAddress( const FSVONodeAddress & address ) const
{
    return address.LayerIndex < 15
               ? SVOData.GetLayer( address.LayerIndex ).GetNode( address.NodeIndex )
               : SVOData.GetLastLayer().GetNode( 0 );
}

FORCEINLINE TSubclassOf< USVONavigationQueryFilter > FSVOVolumeNavigationData::GetVolumeNavigationQueryFilter() const
{
    return VolumeNavigationQueryFilter;
}

FORCEINLINE void FSVOVolumeNavigationData::SetVolumeNavigationQueryFilter( TSubclassOf< USVONavigationQueryFilter > navigation_query_filter )
{
    VolumeNavigationQueryFilter = navigation_query_filter;
}

FORCEINLINE int FSVOVolumeNavigationData::GetLayerCount() const
{
    return SVOData.GetLayerCount();
}
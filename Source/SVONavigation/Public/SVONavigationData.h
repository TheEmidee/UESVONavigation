#pragma once

#include "SVONavigationTypes.h"

#include <CoreMinimal.h>
#include <GameFramework/Actor.h>

#include "SVONavigationData.generated.h"

class USVONavDataRenderingComponent;
struct FSVONavigationBounds;

struct FSVODataConfig : TSharedFromThis< FSVODataConfig >
{
    TEnumAsByte< ECollisionChannel > CollisionChannel;
    float Clearance = 0.0f;
    TWeakObjectPtr< UWorld > World;
    FCollisionQueryParams CollisionQueryParameters;
};

struct FSVOOctreeLeaf
{
    bool GetSubNodeAt( uint_fast32_t X, uint_fast32_t Y, uint_fast32_t Z ) const;
    void SetSubNodeAt( uint_fast32_t X, uint_fast32_t Y, uint_fast32_t Z );

    void SetSubNode( const SubNodeIndex index );
    bool GetSubNode( const MortonCode morton_code ) const;
    void ClearSubNode( const SubNodeIndex index );
    bool IsOccluded() const;
    bool IsEmpty() const;

    uint_fast64_t SubNodes = 0;
};

FORCEINLINE void FSVOOctreeLeaf::SetSubNode( const SubNodeIndex index )
{
    SubNodes |= 1ULL << index;
}

FORCEINLINE bool FSVOOctreeLeaf::GetSubNode( const MortonCode morton_code ) const
{
    return ( SubNodes & 1ULL << morton_code ) != 0;
}

FORCEINLINE void FSVOOctreeLeaf::ClearSubNode( const SubNodeIndex index )
{
    SubNodes &= !( 1ULL << index );
}

FORCEINLINE bool FSVOOctreeLeaf::IsOccluded() const
{
    return SubNodes == -1;
}

FORCEINLINE bool FSVOOctreeLeaf::IsEmpty() const
{
    return SubNodes == 0;
}

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVOOctreeLeaf & data )
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
    {}

    FSVOOctreeLink( const LayerIndex layer_index, const MortonCode node_index, const SubNodeIndex sub_node_index ) :
        LayerIndex( layer_index ),
        NodeIndex( node_index ),
        SubNodeIndex( sub_node_index )
    {}

    bool IsValid() const;
    void Invalidate();

    static FSVOOctreeLink InvalidEdge()
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
    return *( uint32 * ) &link;
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
        Parent( FSVOOctreeLink::InvalidEdge() ),
        FirstChild( FSVOOctreeLink::InvalidEdge() )
    {}

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

struct FSVOOctreeData
{
    void Reset();

    TArray< TArray< FSVOOctreeNode > > NodesByLayers;
    TArray< FSVOOctreeLeaf > Leaves;
};

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVOOctreeData & data )
{
    archive << data.NodesByLayers;
    archive << data.Leaves;

    return archive;
}

USTRUCT()
struct SVONAVIGATION_API FSVONavigationBoundsData
{
    GENERATED_USTRUCT_BODY()

    friend FArchive & operator<<( FArchive & archive, FSVONavigationBoundsData & data );

    const FBox & GetBox() const;
    const FBox & GetVolumeBox() const;
    const FSVOOctreeData & GetOctreeData() const;
    FVector GetNodePosition( LayerIndex layer_index, MortonCode morton_code ) const;
    FVector GetNodePositionFromLink( const FSVOOctreeLink & link ) const;
    float GetLayerVoxelSize( LayerIndex layer_index ) const;
    float GetLayerVoxelHalfSize( LayerIndex layer_index ) const;

    void ComputeDataFromNavigationBounds( const FSVONavigationBounds & navigation_bounds, const FSVODataConfig & config );

private:
    uint32 GetLayerNodeCount( LayerIndex layer_index ) const;
    const TArray< FSVOOctreeNode > & GetLayerNodes( LayerIndex layer_index ) const;
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

    UPROPERTY( VisibleAnywhere, Category = "SVONavigation" )
    int VoxelExponent;

    UPROPERTY( VisibleAnywhere, Category = "SVONavigation" )
    uint8 LayerCount = 0;

    UPROPERTY( VisibleAnywhere, Category = "SVONavigation" )
    FBox VolumeBox;

    UPROPERTY( VisibleAnywhere, Category = "SVONavigation" )
    FBox Box;

    UPROPERTY( VisibleAnywhere, Category = "SVONavigation" )
    float UsedBoxExtent;

    // temporary data used during nav data generation first pass rasterize
    TArray< TSet< MortonCode > > BlockedIndices;
    TArray< float > LayerVoxelSizes;
    TArray< float > LayerVoxelHalfSizes;
    TArray< uint32 > LayerNodeCount;
    FSVOOctreeData OctreeData;
    TWeakObjectPtr< UWorld > World;
    TWeakPtr< const FSVODataConfig > Config;
};

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVONavigationBoundsData  & data )
{
    archive << data.Box;
    archive << data.VolumeBox;
    archive << data.VoxelExponent;
    archive << data.LayerCount;
    archive << data.UsedBoxExtent;
    archive << data.LayerVoxelSizes;
    archive << data.LayerVoxelHalfSizes;
    archive << data.LayerNodeCount;
    archive << data.OctreeData;

    return archive;
}

FORCEINLINE const FBox & FSVONavigationBoundsData::GetBox() const
{
    return Box;
}

FORCEINLINE const FBox & FSVONavigationBoundsData::GetVolumeBox() const
{
    return VolumeBox;
}

FORCEINLINE const FSVOOctreeData & FSVONavigationBoundsData::GetOctreeData() const
{
    return OctreeData;
}

FORCEINLINE float FSVONavigationBoundsData::GetLayerVoxelSize( LayerIndex layer_index ) const
{
    return LayerVoxelSizes[ layer_index ];
}

FORCEINLINE float FSVONavigationBoundsData::GetLayerVoxelHalfSize( LayerIndex layer_index ) const
{
    return LayerVoxelHalfSizes[ layer_index ];
}

FORCEINLINE uint32 FSVONavigationBoundsData::GetLayerNodeCount( LayerIndex layer_index ) const
{
    return LayerNodeCount[ layer_index ];
}

FORCEINLINE const TArray< FSVOOctreeNode > & FSVONavigationBoundsData::GetLayerNodes( LayerIndex layer_index ) const
{
    return OctreeData.NodesByLayers[ layer_index ];
}

FORCEINLINE TArray< FSVOOctreeNode > & FSVONavigationBoundsData::GetLayerNodes( LayerIndex layer_index )
{
    return OctreeData.NodesByLayers[ layer_index ];
}

FORCEINLINE int32 FSVONavigationBoundsData::GetLayerMaxNodeCount( LayerIndex layer_index ) const
{
    return FMath::Pow( 2, VoxelExponent - layer_index );
}

UCLASS( hidecategories = ( Input, Physics, Collisions, Lighting, Rendering, Tags, "Utilities|Transformation", Actor, Layers, Replication ), notplaceable )
class SVONAVIGATION_API ASVONavigationData : public AActor
{
    GENERATED_BODY()

public:
    ASVONavigationData();

    const FSVONavigationBoundsDataDebugInfos & GetDebugInfos() const;

    const TMap< uint32, FSVONavigationBoundsData > & GetNavigationBoundsData() const;

    void PostRegisterAllComponents() override;
    void Serialize( FArchive & archive ) override;

    void AddNavigationBounds( const FSVONavigationBounds & navigation_bounds );
    void UpdateNavigationBounds( const FSVONavigationBounds & navigation_bounds );
    void RemoveNavigationBounds( const FSVONavigationBounds & navigation_bounds );

private:
    UPROPERTY( BlueprintReadOnly, VisibleAnywhere, meta = ( AllowPrivateAccess = true ) )
    USVONavDataRenderingComponent * RenderingComponent;

    UPROPERTY( VisibleAnywhere )
    TMap< uint32, FSVONavigationBoundsData > NavigationBoundsData;

    UPROPERTY( EditInstanceOnly )
    FSVONavigationBoundsDataDebugInfos DebugInfos;

    TSharedPtr< FSVODataConfig > Config;
};

FORCEINLINE const TMap< uint32, FSVONavigationBoundsData > & ASVONavigationData::GetNavigationBoundsData() const
{
    return NavigationBoundsData;
}

FORCEINLINE const FSVONavigationBoundsDataDebugInfos & ASVONavigationData::GetDebugInfos() const
{
    return DebugInfos;
}
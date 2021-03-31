#pragma once

#include "SVONavigationTypes.h"

#include <CoreMinimal.h>


#include "Chaos/AABB.h"
#include "Chaos/AABB.h"

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

struct FSVOOctreeEdge
{
    FSVOOctreeEdge() :
        LayerIndex( 15 ),
        NodeIndex( 0 ),
        SubNodeIndex( 0 )
    {}

    FSVOOctreeEdge( const LayerIndex layer_index, const MortonCode node_index, const SubNodeIndex sub_node_index ) :
        LayerIndex( layer_index ),
        NodeIndex( node_index ),
        SubNodeIndex( sub_node_index )
    {}

    bool IsValid() const;
    void Invalidate();

    static FSVOOctreeEdge InvalidEdge()
    {
        return FSVOOctreeEdge();
    }

    uint8 LayerIndex : 4;
    uint_fast32_t NodeIndex : 22;
    uint8 SubNodeIndex : 6;
};

FORCEINLINE bool FSVOOctreeEdge::IsValid() const
{
    return LayerIndex != 15;
}

FORCEINLINE void FSVOOctreeEdge::Invalidate()
{
    LayerIndex = 15;
}

struct FSVOOctreeNode
{
    MortonCode MortonCode;
    FSVOOctreeEdge Parent;
    FSVOOctreeEdge FirstChild;
    FSVOOctreeEdge AdjacentEdges[ 6 ];

    FSVOOctreeNode() :
        MortonCode( 0 ),
        Parent( FSVOOctreeEdge::InvalidEdge() ),
        FirstChild( FSVOOctreeEdge::InvalidEdge() )
    {}

    /*bool HasChildren() const
    {
        return FirstChild.IsValid();
    }*/
};

struct FSVOOctreeData
{
    void Reset();

    TArray< TArray< FSVOOctreeNode > > NodesByLayers;
    TArray< FSVOOctreeLeaf > Leaves;
};

USTRUCT()
struct SVONAVIGATION_API FSVONavigationBoundsData
{
    GENERATED_USTRUCT_BODY()

    const FBox & GetBox() const;

    void ComputeDataFromNavigationBounds( const FSVONavigationBounds & navigation_bounds, const FSVODataConfig & config );

private:

    uint32 GetLayerNodeCount( LayerIndex layer_index ) const;
    float GetLayerVoxelSize( LayerIndex layer_index ) const;
    float GetLayerVoxelHalfSize( LayerIndex layer_index ) const;
    const TArray< FSVOOctreeNode > & GetOctreeNodesFromLayer( LayerIndex layer_index ) const;
    TArray< FSVOOctreeNode > & GetOctreeNodesFromLayer( LayerIndex layer_index );
    FVector GetNodePosition( LayerIndex layer_index, MortonCode morton_code ) const;
    bool IsPositionOccluded( const FVector & position, float box_size ) const;
    void FirstPassRasterization();
    void AllocateLeafNodes();
    void RasterizeLeaf( const FVector & node_position, int32 leaf_index );
    void RasterizeInitialLayer();
    void RasterizeLayer( LayerIndex layer_index );
    TOptional< NodeIndex > GetNodeIndexFromMortonCode( LayerIndex layer_index, MortonCode morton_code ) const;
    void BuildNeighborLinks( LayerIndex Layer_Index );

    UPROPERTY( VisibleAnywhere, Category = "SVONavigation" )
    int VoxelExponent;

    UPROPERTY( VisibleAnywhere, Category = "SVONavigation" )
    uint8 LayerCount = 0;

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

FORCEINLINE const FBox & FSVONavigationBoundsData::GetBox() const
{
    return Box;
}

FORCEINLINE uint32 FSVONavigationBoundsData::GetLayerNodeCount( LayerIndex layer_index ) const
{
    return LayerNodeCount[ layer_index ];
}

FORCEINLINE float FSVONavigationBoundsData::GetLayerVoxelSize( LayerIndex layer_index ) const
{
    return LayerVoxelSizes[ layer_index ];
}

FORCEINLINE float FSVONavigationBoundsData::GetLayerVoxelHalfSize( LayerIndex layer_index ) const
{
    return LayerVoxelHalfSizes[ layer_index ];
}

FORCEINLINE const TArray< FSVOOctreeNode > & FSVONavigationBoundsData::GetOctreeNodesFromLayer( LayerIndex layer_index ) const
{
    return OctreeData.NodesByLayers[ layer_index ];
}

FORCEINLINE TArray< FSVOOctreeNode > & FSVONavigationBoundsData::GetOctreeNodesFromLayer( LayerIndex layer_index )
{
    return OctreeData.NodesByLayers[ layer_index ];
}

UCLASS( hidecategories = ( Input, Physics, Collisions, Lighting, Rendering, Tags, "Utilities|Transformation", Actor, Layers, Replication ), notplaceable )
class SVONAVIGATION_API ASVONavigationData : public AActor
{
    GENERATED_BODY()

public:
    ASVONavigationData();

    bool HasDebugDrawingEnabled() const;
    const TMap< uint32, FSVONavigationBoundsData > & GetNavigationBoundsData() const;

    void PostRegisterAllComponents() override;

    void AddNavigationBounds( const FSVONavigationBounds & navigation_bounds );
    void UpdateNavigationBounds( const FSVONavigationBounds & navigation_bounds );
    void RemoveNavigationBounds( const FSVONavigationBounds & navigation_bounds );

private:

    UPROPERTY( BlueprintReadOnly, VisibleAnywhere, meta = ( AllowPrivateAccess = true ) )
    USVONavDataRenderingComponent * RenderingComponent;

    UPROPERTY( EditInstanceOnly )
    uint8 ItHasDebugDrawingEnabled : 1;

    UPROPERTY( VisibleAnywhere )
    TMap< uint32, FSVONavigationBoundsData > NavigationBoundsData;

    TSharedPtr< FSVODataConfig > Config;
};

FORCEINLINE bool ASVONavigationData::HasDebugDrawingEnabled() const
{
    return ItHasDebugDrawingEnabled;
}

FORCEINLINE const TMap< uint32, FSVONavigationBoundsData > & ASVONavigationData::GetNavigationBoundsData() const
{
    return NavigationBoundsData;
}
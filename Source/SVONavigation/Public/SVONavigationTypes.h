#pragma once

#include <NavigationSystemTypes.h>
#include "SVONavigationTypes.generated.h"

class ASVONavigationData;
typedef uint_fast64_t MortonCode;
typedef uint8 LayerIndex;
typedef int32 NodeIndex;
typedef uint8 SubNodeIndex;
typedef uint8 NeighborDirection;

DECLARE_DELEGATE_ThreeParams( FSVONavigationPathQueryDelegate, uint32, ENavigationQueryResult::Type, FNavPathSharedPtr );

struct FSVOPathFindingResult
{
    FNavPathSharedPtr Path;
    ENavigationQueryResult::Type Result;

    explicit FSVOPathFindingResult( ENavigationQueryResult::Type result = ENavigationQueryResult::Invalid ) :
        Result( result )
    {}

    bool IsSuccessful() const;
};

FORCEINLINE bool FSVOPathFindingResult::IsSuccessful() const
{
    return Result == ENavigationQueryResult::Success;
}

struct SVONAVIGATION_API FSVOPathFindingQuery : public FPathFindingQueryData
{
    FSVOPathFindingQuery()
    {}

    FSVOPathFindingQuery( const UObject * owner, const ASVONavigationData & navigation_data, const FVector & start, const FVector & end, FNavPathSharedPtr path_to_fill = nullptr ) :
        FPathFindingQueryData( owner, start, end ),
        NavigationData( &navigation_data ),
        PathInstanceToFill( path_to_fill )
    {}

    TWeakObjectPtr< const ASVONavigationData > NavigationData;
    FNavPathSharedPtr PathInstanceToFill;
};

struct SVONAVIGATION_API FSVOAsyncPathFindingQuery : public FSVOPathFindingQuery
{
    const uint32 QueryID;
    const FSVONavigationPathQueryDelegate OnDoneDelegate;
    FSVOPathFindingResult Result;

    FSVOAsyncPathFindingQuery() :
        QueryID( INVALID_NAVQUERYID )
    {}

    FSVOAsyncPathFindingQuery( const FSVOPathFindingQuery & path_finding_query, const FSVONavigationPathQueryDelegate & on_done_delegate ) :
        FSVOPathFindingQuery( path_finding_query ),
        QueryID( GetUniqueID() ),
        OnDoneDelegate( on_done_delegate )
    {
    }

protected:
    FORCEINLINE static uint32 GetUniqueID()
    {
        return ++LastPathFindingUniqueID;
    }

    static uint32 LastPathFindingUniqueID;
};

uint32 FSVOAsyncPathFindingQuery::LastPathFindingUniqueID = 0;

USTRUCT()
struct SVONAVIGATION_API FSVONavigationBoundsDataDebugInfos
{
    GENERATED_USTRUCT_BODY()

    FSVONavigationBoundsDataDebugInfos() :
        ItHasDebugDrawingEnabled( false ),
        ItDebugDrawsBounds( false ),
        ItDebugDrawsLayers( false ),
        LayerIndexToDraw( 1 ),
        ItDebugDrawsLeaves( false ),
        ItDebugDrawsOccludedLeaves( false ),
        ItDebugDrawsLinks( false ),
        LinksLayerIndexToDraw( false ),
        DebugLineThickness( 5.0f )
    {}

    friend FArchive & operator<<( FArchive & archive, FSVONavigationBoundsDataDebugInfos & data );

    UPROPERTY( EditInstanceOnly )
    bool ItHasDebugDrawingEnabled;

    UPROPERTY( EditInstanceOnly, meta = ( EditCondition = "ItHasDebugDrawingEnabled" ) )
    bool ItDebugDrawsBounds;

    UPROPERTY( EditInstanceOnly, meta = ( EditCondition = "ItHasDebugDrawingEnabled" ) )
    bool ItDebugDrawsLayers;

    UPROPERTY( EditInstanceOnly, meta = ( EditCondition = "ItHasDebugDrawingEnabled && ItDebugDrawsLayers", ClampMin = "1", UIMin = "1" ) )
    uint8 LayerIndexToDraw;

    UPROPERTY( EditInstanceOnly, meta = ( EditCondition = "ItHasDebugDrawingEnabled" ) )
    bool ItDebugDrawsLeaves;

    UPROPERTY( EditInstanceOnly, meta = ( EditCondition = "ItHasDebugDrawingEnabled" ) )
    bool ItDebugDrawsOccludedLeaves;

    UPROPERTY( EditInstanceOnly, meta = ( EditCondition = "ItHasDebugDrawingEnabled" ) )
    bool ItDebugDrawsLinks;

    UPROPERTY( EditInstanceOnly, meta = ( EditCondition = "ItHasDebugDrawingEnabled && ItDebugDrawsLinks", ClampMin = "1", UIMin = "1" ) )
    uint8 LinksLayerIndexToDraw;

    UPROPERTY( EditInstanceOnly, meta = ( EditCondition = "ItHasDebugDrawingEnabled && ItDebugDrawsLinks" ) )
    bool ItDebugDrawsNeighborLinks;

    UPROPERTY( EditInstanceOnly, meta = ( EditCondition = "ItHasDebugDrawingEnabled && ItDebugDrawsLinks" ) )
    bool ItDebugDrawsParentLinks;

    UPROPERTY( EditInstanceOnly, meta = ( EditCondition = "ItHasDebugDrawingEnabled && ItDebugDrawsLinks" ) )
    bool ItDebugDrawsFirstChildLinks;

    UPROPERTY( EditInstanceOnly, meta = ( EditCondition = "ItHasDebugDrawingEnabled", ClampMin = "1", UIMin = "1" ) )
    float DebugLineThickness;
};

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVONavigationBoundsDataDebugInfos & data )
{
    archive << data.ItHasDebugDrawingEnabled;
    archive << data.ItDebugDrawsBounds;
    archive << data.ItDebugDrawsLayers;
    archive << data.LayerIndexToDraw;
    archive << data.ItDebugDrawsLeaves;
    archive << data.ItDebugDrawsOccludedLeaves;
    archive << data.ItDebugDrawsLinks;
    archive << data.LinksLayerIndexToDraw;
    archive << data.ItDebugDrawsNeighborLinks;
    archive << data.ItDebugDrawsParentLinks;
    archive << data.DebugLineThickness;

    return archive;
}

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
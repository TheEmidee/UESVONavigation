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
struct FSVODataBuildConfig
{
    GENERATED_USTRUCT_BODY()

    UPROPERTY( EditInstanceOnly, Category = "Generation" )
    TEnumAsByte< ECollisionChannel > CollisionChannel;

    UPROPERTY( EditInstanceOnly, Category = "Generation" )
    float Clearance = 0.0f;

    FCollisionQueryParams CollisionQueryParameters;
};
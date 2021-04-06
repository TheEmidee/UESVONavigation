#pragma once

#include "SVONavigationTypes.generated.h"

typedef uint_fast64_t MortonCode;
typedef uint8 LayerIndex;
typedef int32 NodeIndex;
typedef uint8 SubNodeIndex;
typedef uint8 NeighborDirection;

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
#pragma once

#include <CoreMinimal.h>
#include <NavigationData.h>

#include "SVONavigationDataDetails.generated.h"

USTRUCT()
struct SVONAVIGATION_API FSVOVolumeNavigationDataDebugInfos
{
    GENERATED_USTRUCT_BODY()

    FSVOVolumeNavigationDataDebugInfos();

    UPROPERTY( EditInstanceOnly, meta = ( Tooltip = "Draws the bounding box of the entire SVO volume." ) )
    uint8 bDebugDrawBounds : 1;

    UPROPERTY( EditInstanceOnly, meta = ( Tooltip = "Draws the integer coordinates (e.g., 0,1,0) of a node within its layer." ) )
    uint8 bDebugDrawNodeCoords : 1;

    UPROPERTY( EditInstanceOnly, meta = ( Tooltip = "Draws the Morton Code of a node, which represents its position in a Z-order curve." ) )
    uint8 bDebugDrawMortonCoords : 1;

    UPROPERTY( EditInstanceOnly, meta = ( Tooltip = "Draws the full node address (Layer:Node:SubNode)." ) )
    uint8 bDebugDrawNodeAddresses : 1;

    UPROPERTY( EditInstanceOnly, meta = ( Tooltip = "Draws the FVector world-space location of the node's center." ) )
    uint8 bDebugDrawNodeLocation : 1;

    UPROPERTY( EditInstanceOnly, meta = ( Tooltip = "Master switch to enable drawing of a specific SVO layer." ) )
    uint8 bDebugDrawLayers : 1;

    UPROPERTY( EditInstanceOnly, meta = ( EditCondition = "bDebugDrawLayers", ClampMin = "0", UIMin = "0", Tooltip = "Specifies which layer to draw if Debug Draw Layers is enabled." ) )
    uint8 LayerIndexToDraw;

    UPROPERTY( EditInstanceOnly, meta = ( Tooltip = "Master switch to enable drawing of the highest-resolution leaf voxels (sub-nodes)." ) )
    uint8 bDebugDrawSubNodes : 1;

    UPROPERTY( EditInstanceOnly, meta = ( Tooltip = "Draws voxels that are blocked by geometry (Orange)." ) )
    uint8 bDebugDrawOccludedVoxels : 1;

    UPROPERTY( EditInstanceOnly, meta = ( Tooltip = "Draws voxels that are navigable (Green)." ) )
    uint8 bDebugDrawFreeVoxels : 1;

    UPROPERTY( EditInstanceOnly, meta = ( Tooltip = "Visualizes the pathfinding connections between nodes." ) )
    uint8 bDebugDrawNeighborLinks : 1;

    UPROPERTY( EditInstanceOnly, meta = ( Tooltip = "For a specific node address (e.g., '1 12 0'), shows its neighbor links." ) )
    FString NeighborLinksForNodeAddress;

    UPROPERTY( EditInstanceOnly, meta = ( Tooltip = "Draws the paths currently being calculated or followed by AI agents." ) )
    uint8 bDebugDrawActivePaths : 1;
};

USTRUCT()
struct SVONAVIGATION_API FSVONavigationDataInfos
{
    GENERATED_USTRUCT_BODY()

    FSVONavigationDataInfos() :
        VolumeLocation( ForceInit ),
        bHasNavigationData( false ),
        LayerCount( INDEX_NONE )
    {
    }

    UPROPERTY( VisibleInstanceOnly )
    FVector VolumeLocation;

    UPROPERTY( VisibleInstanceOnly )
    uint8 bHasNavigationData : 1;

    UPROPERTY( VisibleInstanceOnly )
    int LayerCount;
};

USTRUCT()
struct SVONAVIGATION_API FSVODataInfos
{
    GENERATED_USTRUCT_BODY()

    UPROPERTY( EditInstanceOnly )
    TArray< FSVONavigationDataInfos > Infos;
};
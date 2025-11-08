#include "SVONavigationDataDetails.h"

FSVOVolumeNavigationDataDebugInfos::FSVOVolumeNavigationDataDebugInfos() :
    bDebugDrawBounds( false ),
    bDebugDrawNodeCoords( false ),
    bDebugDrawMortonCoords( false ),
    bDebugDrawNodeAddresses( false ),
    bDebugDrawNodeLocation( false ),
    bDebugDrawLayers( false ),
    LayerIndexToDraw( 0 ),
    bDebugDrawSubNodes( false ),
    bDebugDrawOccludedVoxels( true ),
    bDebugDrawFreeVoxels( false ),
    bDebugDrawNeighborLinks( false ),
    bDebugDrawActivePaths( false )
{
}
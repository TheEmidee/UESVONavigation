#pragma once

enum class ESVOVersion : uint8
{
    Initial = 1,
    NoVoxelExponent = 2,
    LeafNodeParent = 3,
    VolumeNavigationQueryFilter = 4,
    NavigationDataChunks = 5,

    MinCompatible = NavigationDataChunks,
    Latest = NavigationDataChunks
};
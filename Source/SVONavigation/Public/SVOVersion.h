#pragma once

enum class ESVOVersion : uint8
{
    Initial = 1,
    NoVoxelExponent = 2,

    MinCompatible = NoVoxelExponent,
    Latest = NoVoxelExponent
};
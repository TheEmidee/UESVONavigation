#pragma once

#include <CoreMinimal.h>

#include <libmorton/morton.h>

#include "SVONavigationTypes.h"

class SVONAVIGATION_API FSVOHelpers
{
public:

    template< typename _VECTOR_ >
    FORCEINLINE static MortonCode GetMortonCodeFromVector( const _VECTOR_ & vector )
    {
        return morton3D_64_encode( vector.X, vector.Y, vector.Z );
    }

    FORCEINLINE static FVector GetVectorFromMortonCode( const MortonCode morton_code )
    {
        uint_fast32_t x, y, z;
        morton3D_64_decode( morton_code, x, y, z );

        return FVector( x, y, z );
    }

    FORCEINLINE static MortonCode GetParentMortonCode( const MortonCode child_morton_code )
    {
        return child_morton_code >> 3;
    }

    FORCEINLINE static MortonCode GetFirstChildMortonCode( const MortonCode parent_morton_code )
    {
        return parent_morton_code << 3;
    }
};

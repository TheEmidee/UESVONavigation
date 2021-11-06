#pragma once

#include <CoreMinimal.h>

#include <libmorton/morton.h>

#include "SVONavigationTypes.h"

class SVONAVIGATION_API FSVOHelpers
{
public:

    template< typename _VECTOR_ >
    static MortonCode GetMortonCodeFromVector( const _VECTOR_ & vector )
    {
        return morton3D_64_encode( vector.X, vector.Y, vector.Z );
    }

    static FVector GetVectorFromMortonCode( const MortonCode morton_code )
    {
        uint_fast32_t x, y, z;
        morton3D_64_decode( morton_code, x, y, z );

        return FVector( x, y, z );
    }
};

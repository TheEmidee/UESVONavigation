#pragma once

#include "SVONavigationTypes.h"

#include <CoreMinimal.h>
#include <GraphAStar.h>
#include <ThirdParty/libmorton/morton.h>

class SVONAVIGATION_API FSVOHelpers
{
public:
    FORCEINLINE static MortonCode GetMortonCodeFromVector( const FVector & vector )
    {
        return morton3D_64_encode( vector.X, vector.Y, vector.Z );
    }

    FORCEINLINE static MortonCode GetMortonCodeFromVector( const FIntVector & vector )
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

    FORCEINLINE static ENavigationQueryResult::Type GraphAStarResultToNavigationTypeResult( const EGraphAStarResult result )
    {
        constexpr ENavigationQueryResult::Type result_conversion_table[] = {
            ENavigationQueryResult::Fail,
            ENavigationQueryResult::Success,
            ENavigationQueryResult::Fail,
            ENavigationQueryResult::Fail
        };

        return result_conversion_table[ static_cast< int >( result ) ];
    }
};

#pragma once

#include <CoreMinimal.h>

#include "SVOBoundsNavigationData.h"

struct FSVOData
{
public:

    const TArray< FSVOBoundsNavigationData > & GetNavigationBoundsData() const;

    void Serialize( FArchive & archive );
    void ClearData();
    void RemoveDataInBounds( const FBox & bounds );
    void AddNavigationBoundsData( FSVOBoundsNavigationData && data );
    FBox GetBoundingBox() const;

#if !UE_BUILD_SHIPPING
    uint32 GetAllocatedSize() const;
#endif

private:

	TArray< FSVOBoundsNavigationData > NavigationBoundsData;
};

typedef TUniquePtr< FSVOData > FUniqueSVODataPtr;

FORCEINLINE const TArray< FSVOBoundsNavigationData > & FSVOData::GetNavigationBoundsData() const
{
    return NavigationBoundsData;
}
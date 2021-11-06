#include "SVOData.h"

void FSVOData::Serialize( FArchive & archive, const ESVOVersion version )
{
    auto bounds_count = NavigationBoundsData.Num();
    archive << bounds_count;
    if ( archive.IsLoading() )
    {
        NavigationBoundsData.Reset( bounds_count );
        NavigationBoundsData.SetNum( bounds_count );
    }

    for ( auto index = 0; index < bounds_count; index++ )
    {
        NavigationBoundsData[ index ].Serialize( archive, version );
    }
}

void FSVOData::ClearData()
{
    NavigationBoundsData.Reset();
}

void FSVOData::RemoveDataInBounds( const FBox & bounds )
{
    NavigationBoundsData.RemoveAll( [ &bounds ]( const FSVOBoundsNavigationData & data ) {
        return data.GetVolumeBounds() == bounds;
    } );
}

void FSVOData::AddNavigationBoundsData( FSVOBoundsNavigationData data )
{
    NavigationBoundsData.Emplace( MoveTemp( data ) );
}

FBox FSVOData::GetBoundingBox() const
{
    FBox bounding_box( ForceInit );

    for ( const auto & bounds : NavigationBoundsData )
    {
        bounding_box += bounds.GetNavigationBounds();
    }

    return bounding_box;
}

const FSVOBoundsNavigationData * FSVOData::GetBoundsNavigationDataContainingPoints( const TArray< FVector > & points ) const
{
    return NavigationBoundsData.FindByPredicate( [ this, &points ]( const FSVOBoundsNavigationData & data ) {
        const auto & bounds = data.GetNavigationBounds();
        for ( const auto & point : points )
        {
            if ( !bounds.IsInside( point ) )
            {
                return false;
            }
        }
        return true;
    } );
}

#if !UE_BUILD_SHIPPING
uint32 FSVOData::GetAllocatedSize() const
{
    auto navigation_mem_size = 0;
    for ( const auto & nav_bounds_data : NavigationBoundsData )
    {
        const auto octree_data_mem_size = nav_bounds_data.GetOctreeData().GetAllocatedSize();
        navigation_mem_size += octree_data_mem_size;
    }
    return navigation_mem_size;
}
#endif
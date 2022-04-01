#include "SVONavigationDataChunk.h"

#include "SVOVersion.h"

void USVONavigationDataChunk::Serialize( FArchive & archive )
{
    Super::Serialize( archive );

    ESVOVersion version = ESVOVersion::Latest;
    archive << version;

    // when writing, write a zero here for now.  will come back and fill it in later.
    auto svo_size_bytes = 0;
    const auto svo_size_position = archive.Tell();

    archive << svo_size_bytes;

    if ( archive.IsLoading() )
    {
        if ( version < ESVOVersion::MinCompatible )
        {
            // incompatible, just skip over this data
            archive.Seek( svo_size_position + svo_size_bytes );
            return;
        }
    }

    auto volume_count = NavigationData.Num();
    archive << volume_count;
    if ( archive.IsLoading() )
    {
        NavigationData.Reset( volume_count );
        NavigationData.SetNum( volume_count );
    }

    for ( auto index = 0; index < volume_count; index++ )
    {
        NavigationData[ index ].Serialize( archive, version );
    }

    if ( archive.IsSaving() )
    {
        const auto current_position = archive.Tell();

        svo_size_bytes = current_position - svo_size_position;

        archive.Seek( svo_size_position );
        archive << svo_size_bytes;
        archive.Seek( current_position );
    }
}

void USVONavigationDataChunk::AddNavigationData( FSVOVolumeNavigationData & navigation_data )
{
    navigation_data.SetInNavigationDataChunk( true );
    NavigationData.Emplace( navigation_data );
}

void USVONavigationDataChunk::ReleaseNavigationData()
{
    NavigationData.Reset();
}

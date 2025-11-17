#include "SVOVolumeNavigationData.h"
#include "SVOVersion.h"

void FSVOVolumeNavigationData::Serialize( FArchive & archive, const ESVOVersion version )
{
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

	archive << VolumeBounds;
	archive << SVOData;
	archive << VolumeNavigationQueryFilter;
	archive << bInNavigationDataChunk;

	if ( archive.IsSaving() )
	{
		const auto current_position = archive.Tell();

		svo_size_bytes = current_position - svo_size_position;

		archive.Seek( svo_size_position );
		archive << svo_size_bytes;
		archive.Seek( current_position );
	}
}
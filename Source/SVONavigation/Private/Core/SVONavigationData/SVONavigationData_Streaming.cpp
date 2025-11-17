#include "SVONavigationData.h"
#include "SVONavigationDataChunk.h"
#include "NavigationSystem.h"

bool ASVONavigationData::SupportsStreaming() const
{
    return ( RuntimeGeneration != ERuntimeGenerationType::Dynamic );
}

void ASVONavigationData::OnStreamingLevelAdded( ULevel * level, UWorld * /*world*/ )
{
    // QUICK_SCOPE_CYCLE_COUNTER( STAT_RecastNavMesh_OnStreamingLevelAdded );

    if ( SupportsStreaming() )
    {
        if ( USVONavigationDataChunk * navigation_data_chunk = GetNavigationDataChunk( level ) )
        {
            for ( const auto & chunk_nav_data : navigation_data_chunk->NavigationData )
            {
                if ( VolumeNavigationData.FindByPredicate( [ &chunk_nav_data ]( const auto & navigation_data ) {
                         return chunk_nav_data.GetVolumeBounds() == navigation_data.GetVolumeBounds();
                     } ) == nullptr )
                {
                    VolumeNavigationData.Add( chunk_nav_data );
                }
            }

            RequestDrawingUpdate();
        }
    }
}

void ASVONavigationData::OnStreamingLevelRemoved( ULevel * level, UWorld * /*world*/ )
{
    // QUICK_SCOPE_CYCLE_COUNTER( STAT_RecastNavMesh_OnStreamingLevelRemoved );

    if ( SupportsStreaming() )
    {
        if ( USVONavigationDataChunk * navigation_data_chunk = GetNavigationDataChunk( level ) )
        {
            for ( const auto & chunk_nav_data : navigation_data_chunk->NavigationData )
            {
                VolumeNavigationData.RemoveAllSwap( [ &chunk_nav_data ]( const auto & nav_data ) {
                    return chunk_nav_data.GetVolumeBounds() == nav_data.GetVolumeBounds();
                } );
            }

            RequestDrawingUpdate();
        }
    }
}

void ASVONavigationData::CheckToDiscardSubLevelNavData( const UNavigationSystemBase & navigation_system )
{
    if ( const auto * world = GetWorld() )
    {
        if ( const auto * nav_sys = Cast< UNavigationSystemV1 >( &navigation_system ) )
        {
            // Get rid of instances saved within levels that are streamed-in
            if ( GEngine->IsSettingUpPlayWorld() == false // this is a @HACK
                 && ( world->PersistentLevel != GetLevel() )
                 // If we are cooking, then let them all pass.
                 // They will be handled at load-time when running.
                 && ( IsRunningCommandlet() == false ) )
            {
                UE_LOG( LogNavigation, Verbose, TEXT( "%s Discarding %s due to it not being part of PersistentLevel." ), ANSI_TO_TCHAR( __FUNCTION__ ), *GetFullNameSafe( this ) );

                // Marking self for deletion
                CleanUpAndMarkPendingKill();
            }
        }
    }
}

USVONavigationDataChunk * ASVONavigationData::GetNavigationDataChunk( ULevel * level ) const
{
    const auto this_name = GetFName();

    if ( const auto * result = level->NavDataChunks.FindByPredicate( [ & ]( const UNavigationDataChunk * chunk ) {
             return chunk->NavigationDataName == this_name;
         } ) )
    {
        return Cast< USVONavigationDataChunk >( *result );
    }

    return nullptr;
}
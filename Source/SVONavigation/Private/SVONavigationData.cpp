#include "SVONavigationData.h"

#include "PathFinding/SVONavigationQueryFilterImpl.h"
#include "PathFinding/SVONavigationPathFinderImpl.h"
#include "SVONavigationDataChunk.h"
#include "SVOVersion.h"

#include "AI/NavDataGenerator.h"
#include "NavigationSystem.h"

ASVONavigationData::ASVONavigationData() :
    Version( ESVOVersion::Latest )
{
    MaxSimultaneousBoxGenerationJobsCount = 1024;

    if ( !HasAnyFlags( RF_ClassDefaultObject ) )
    {
        FindPathImplementation = FSVONavigationPathFinderImpl::FindPath;
    }
}

void ASVONavigationData::PostInitProperties()
{
    Super::PostInitProperties();

    if ( HasAnyFlags( RF_ClassDefaultObject | RF_NeedLoad ) == false )
    {
        RecreateDefaultFilter();
    }
}

void ASVONavigationData::PostLoad()
{
    Super::PostLoad();

    if ( const auto * world = GetWorld() )
    {
        const auto * navigation_system_base = world->GetNavigationSystem();
        if ( navigation_system_base != nullptr && navigation_system_base->IsWorldInitDone() )
        {
            CheckToDiscardSubLevelNavData( *navigation_system_base );
        }
        else
        {
            UNavigationSystemBase::OnNavigationInitStartStaticDelegate().AddUObject( this, &ThisClass::CheckToDiscardSubLevelNavData );
        }
    }

    RecreateDefaultFilter();
}

void ASVONavigationData::Serialize( FArchive & archive )
{
    Super::Serialize( archive );

    archive << Version;

    // Same as in RecastNavMesh
    uint32 svo_size_bytes = 0;
    const auto svo_size_position = archive.Tell();

    archive << svo_size_bytes;

    if ( archive.IsLoading() )
    {
        auto clean_up_bad_version = [ &archive, svo_size_bytes, svo_size_position, this ]() {
            // incompatible, just skip over this data.  navmesh needs rebuilt.
            archive.Seek( svo_size_position + svo_size_bytes );

            // Mark self for delete
            CleanUpAndMarkPendingKill();
        };

        if ( Version < ESVOVersion::MinCompatible )
        {
            UE_LOG( LogNavigation, Warning, TEXT( "%s: ASVONavigationData: Nav mesh version %d < Min compatible %d. Nav mesh needs to be rebuilt. \n" ), *GetFullName(), Version, ESVOVersion::MinCompatible );
            // incompatible, just skip over this data.  nav mesh needs rebuilt.
            clean_up_bad_version();
            return;
        }
        if ( Version > ESVOVersion::Latest )
        {
            UE_LOG( LogNavigation, Warning, TEXT( "%s: ASVONavigationData: Nav mesh version %d > NAVMESHVER_LATEST %d. Newer nav mesh should not be loaded by older code. At a minimum the nav mesh needs to be rebuilt. \n" ), *GetFullName(), Version, ESVOVersion::Latest );
            clean_up_bad_version();
            return;
        }
        if ( svo_size_bytes > 4 )
        {
            SerializeSVOData( archive, Version );
#if !( UE_BUILD_SHIPPING )
            RequestDrawingUpdate();
#endif //!(UE_BUILD_SHIPPING)
        }
        else
        {
            // empty, just skip over this data
            archive.Seek( svo_size_position + svo_size_bytes );
            // if it's not getting filled it's better to just remove it
            VolumeNavigationData.Reset();
        }
    }
    else
    {
        SerializeSVOData( archive, Version );

        const int64 current_position = archive.Tell();

        svo_size_bytes = current_position - svo_size_position;

        archive.Seek( svo_size_position );
        archive << svo_size_bytes;
        archive.Seek( current_position );
    }
}

void ASVONavigationData::CleanUp()
{
    Super::CleanUp();
    ResetGenerator();
}

void ASVONavigationData::SerializeSVOData( FArchive & archive, ESVOVersion version )
{
    if ( archive.IsLoading() )
    {
        auto volume_count = VolumeNavigationData.Num();
        archive << volume_count;
        VolumeNavigationData.Reset( volume_count );
        VolumeNavigationData.SetNum( volume_count );

        for ( auto index = 0; index < volume_count; index++ )
        {
            VolumeNavigationData[ index ].Serialize( archive, Version );
        }
    }
    else
    {
        // When saving, don't serialize the whole VolumeNavigationData array as it may contain navigation data from chunks added by streaming levels
        TArray< FSVOVolumeNavigationData > level_volume_navigation_data;

        if ( SupportsStreaming() && FNavigationSystem::GetCurrent< const UNavigationSystemV1 >( GetWorld() ) != nullptr )
        {
            const auto & level_navigable_bounds = GetNavigableBoundsInLevel( GetLevel() );

            TArray< bool > navigation_data_indices_to_keep;
            navigation_data_indices_to_keep.SetNum( VolumeNavigationData.Num() );

            for ( const auto & navigable_bounds : level_navigable_bounds )
            {
                const auto index = VolumeNavigationData.IndexOfByPredicate( [ &navigable_bounds ]( const auto & navigation_data ) {
                    return !navigation_data.IsInNavigationDataChunk() && /*!*/( navigation_data.GetVolumeBounds() == navigable_bounds );
                } );

                if ( index != INDEX_NONE )
                {
                    navigation_data_indices_to_keep[ index ] = true;
                }
            }

            for ( auto index = VolumeNavigationData.Num() - 1; index >= 0; --index )
            {
                if ( navigation_data_indices_to_keep[ index ] )
                {
                    level_volume_navigation_data.Add( VolumeNavigationData[ index ] );
                }
            }
        }
        else
        {
            level_volume_navigation_data = VolumeNavigationData;
        }

        auto volume_count = level_volume_navigation_data.Num();
        archive << volume_count;

        for ( auto index = 0; index < volume_count; index++ )
        {
            level_volume_navigation_data[ index ].Serialize( archive, Version );
        }
    }
}

void ASVONavigationData::RecreateDefaultFilter() const
{
    DefaultQueryFilter->SetFilterType< FSVONavigationQueryFilterImpl >();
}

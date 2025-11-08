#include "SVONavigationData.h"
#include "SVONavigationDataGenerator.h"
#include "SVONavigationDataChunk.h"
#include "SVOBoundsVolume.h"
#include "SVOVersion.h"
#include "SVONavigationSettings.h"

#include <AI/NavDataGenerator.h>
#include <NavigationSystem.h>
#include <EngineUtils.h>

#if WITH_EDITOR
#include <ObjectEditorUtils.h>
#endif

bool ASVONavigationData::NeedsRebuild() const
{
    const auto needs_rebuild = VolumeNavigationData.FindByPredicate( []( const FSVOVolumeNavigationData & data ) {
        return !data.GetData().IsValid();
    } ) != nullptr;

    if ( NavDataGenerator.IsValid() )
    {
        return needs_rebuild || NavDataGenerator->GetNumRemaningBuildTasks() > 0;
    }

    return needs_rebuild;
}

void ASVONavigationData::EnsureBuildCompletion()
{
    Super::EnsureBuildCompletion();

    // Doing this as a safety net solution due to UE-20646, which was basically a result of random
    // over-releasing of default filter's shared pointer (it seemed). We might have time to get
    // back to this time some time in next 3 years :D
    RecreateDefaultFilter();
}

bool ASVONavigationData::SupportsRuntimeGeneration() const
{
    return (RuntimeGeneration == ERuntimeGenerationType::Dynamic);
}

void ASVONavigationData::OnNavAreaChanged()
{
    Super::OnNavAreaChanged();
}

void ASVONavigationData::OnNavAreaAdded( const UClass * nav_area_class, int32 agent_index )
{
    Super::OnNavAreaAdded( nav_area_class, agent_index );
}

int32 ASVONavigationData::GetNewAreaID( const UClass * nav_area_class ) const
{
    return Super::GetNewAreaID( nav_area_class );
}

int32 ASVONavigationData::GetMaxSupportedAreas() const
{
    return 32;
}

#if WITH_EDITOR
void ASVONavigationData::PostEditChangeProperty( FPropertyChangedEvent & property_changed_event )
{
    Super::PostEditChangeProperty( property_changed_event );

    if ( property_changed_event.Property == nullptr )
    {
        return;
    }

    if ( property_changed_event.Property != nullptr )
    {
        const FName category_name = FObjectEditorUtils::GetCategoryFName( property_changed_event.Property );
        static const FName NAME_Generation = FName( TEXT( "Generation" ) );
        static const FName NAME_Query = FName( TEXT( "Query" ) );

        if ( category_name == NAME_Generation )
        {
            if ( auto * settings = GetDefault< USVONavigationSettings >() )
            {
                if ( !HasAnyFlags( RF_ClassDefaultObject ) && settings->bNavigationAutoUpdateEnabled )
                {
                    RebuildAll();
                }
            }
        }
        else if ( category_name == NAME_Query )
        {
            RecreateDefaultFilter();
        }
    }
}

bool ASVONavigationData::ShouldExport()
{
    return false;
}
#endif

void ASVONavigationData::ConditionalConstructGenerator()
{
    ResetGenerator();

    UWorld * world = GetWorld();
    check( world );
    const bool requires_generator = SupportsRuntimeGeneration() || !world->IsGameWorld();

    if ( !requires_generator )
    {
        return;
    }

    if ( FSVONavigationDataGenerator * generator = new FSVONavigationDataGenerator( *this ) )
    {
        NavDataGenerator = MakeShareable( static_cast< FNavDataGenerator * >( generator ) );
        generator->Init();
    }
}

void ASVONavigationData::RemoveDataInBounds( const FBox & bounds )
{
    VolumeNavigationData.RemoveAllSwap( [ &bounds ]( const FSVOVolumeNavigationData & data ) {
        return data.GetVolumeBounds() == bounds;
    } );
}

void ASVONavigationData::AddVolumeNavigationData( FSVOVolumeNavigationData data )
{
    for ( TActorIterator< ASVOBoundsVolume > iterator( GetWorld(), ASVOBoundsVolume::StaticClass() ); iterator; ++iterator )
    {
        const auto * volume = *iterator;

        if ( volume->GetComponentsBoundingBox( true ) == data.GetVolumeBounds() )
        {
            data.SetVolumeNavigationQueryFilter( volume->GetVolumeNavigationQueryFilter() );
            break;
        }
    }

    VolumeNavigationData.Emplace( MoveTemp( data ) );
}

void ASVONavigationData::UpdateNavVersion()
{
    Version = ESVOVersion::Latest;
}

void ASVONavigationData::ResetGenerator( const bool cancel_build )
{
    if ( NavDataGenerator.IsValid() )
    {
        if ( cancel_build )
        {
            NavDataGenerator->CancelBuild();
        }

        NavDataGenerator.Reset();
    }
}

void ASVONavigationData::OnNavigationDataUpdatedInBounds( const TArray< FBox > & updated_bounds )
{
    InvalidateAffectedPaths( updated_bounds );
}

void ASVONavigationData::ClearNavigationData()
{
    VolumeNavigationData.Reset();
    RequestDrawingUpdate();
}

void ASVONavigationData::BuildNavigationData()
{
    RebuildAll();
}

void ASVONavigationData::InvalidateAffectedPaths( const TArray< FBox > & updated_bounds )
{
    const int32 paths_count = ActivePaths.Num();
    const int32 updated_bounds_count = updated_bounds.Num();

    if ( updated_bounds_count == 0 || paths_count == 0 )
    {
        return;
    }

    // Paths can be registered from async pathfinding thread.
    // Theoretically paths are invalidated synchronously by the navigation system
    // before starting async queries task but protecting ActivePaths will make
    // the system safer in case of future timing changes.
    {
        FScopeLock path_lock( &ActivePathsLock );

        FNavPathWeakPtr * weak_path_ptr = ( ActivePaths.GetData() + paths_count - 1 );

        for ( int32 path_index = paths_count - 1; path_index >= 0; --path_index, --weak_path_ptr )
        {
            FNavPathSharedPtr shared_path = weak_path_ptr->Pin();
            if ( !weak_path_ptr->IsValid() )
            {
                ActivePaths.RemoveAtSwap( path_index, 1, EAllowShrinking::No );
            }
            else
            {
                const FNavigationPath * path = shared_path.Get();
                if ( !path->IsReady() || path->GetIgnoreInvalidation() )
                {
                    // path not filled yet or doesn't care about invalidation
                    continue;
                }

                for ( const auto & path_point : path->GetPathPoints() )
                {
                    if ( updated_bounds.FindByPredicate( [ &path_point ]( const FBox & bounds ) {
                             return bounds.IsInside( path_point.Location );
                         } ) != nullptr )
                    {
                        shared_path->Invalidate();
                        ActivePaths.RemoveAtSwap( path_index, 1, EAllowShrinking::No );

                        break;
                    }
                }

                if ( !shared_path->IsValid() )
                {
                    break;
                }
            }
        }
    }
}

void ASVONavigationData::OnNavigationDataGenerationFinished()
{
    if ( UWorld * world = GetWorld() )
    {
        if ( IsValid( world ) )
        {
#if WITH_EDITOR
            // For navmeshes that support streaming create navigation data holders in each streaming level
            // so parts of navmesh can be streamed in/out with those levels
            if ( !world->IsGameWorld() )
            {
                const auto & levels = world->GetLevels();

                for ( auto * level : levels )
                {
                    if ( level->IsPersistentLevel() )
                    {
                        continue;
                    }

                    USVONavigationDataChunk * navigation_data_chunk = GetNavigationDataChunk( level );

                    if ( SupportsStreaming() )
                    {
                        // We use navigation volumes that belongs to this streaming level to find tiles we want to save
                        const auto & level_nav_bounds = GetNavigableBoundsInLevel( level );

                        TArray< int32 > navigation_data_indices;
                        navigation_data_indices.Reserve( level_nav_bounds.Num() );

                        for ( const auto & nav_bounds : level_nav_bounds )
                        {
                            const auto index = VolumeNavigationData.IndexOfByPredicate( [ &nav_bounds ]( const FSVOVolumeNavigationData & data ) {
                                const auto & bounds = data.GetData().GetVolumeBounds();
                                return bounds == nav_bounds;
                            } );

                            if ( index != INDEX_NONE )
                            {
                                navigation_data_indices.Add( index );
                            }
                        }

                        if ( navigation_data_indices.Num() > 0 )
                        {
                            // Create new chunk only if we have something to save in it
                            if ( navigation_data_chunk == nullptr )
                            {
                                navigation_data_chunk = NewObject< USVONavigationDataChunk >( level );
                                navigation_data_chunk->NavigationDataName = GetFName();
                                level->NavDataChunks.Add( navigation_data_chunk );
                            }

                            for ( const auto index : navigation_data_indices )
                            {
                                navigation_data_chunk->AddNavigationData( VolumeNavigationData[ index ] );
                            }

                            navigation_data_chunk->MarkPackageDirty();
                            continue;
                        }
                    }

                    // It's hack. That check should not be there.
                    // When calling FNavigationSystem::Build, all streaming levels should be loaded and visible for the navigation to be built. That's how it works for ReCast
                    // But since svo nav data always resolves to a box bigger than the nav bounds volume, it's possible that when building navigation for a volume in a streaming
                    // level, the box would encompasses geometry of another level which should not be visible.
                    // The solution we use in our game is to use a BuildIncremental function on a custom navigation system, which never calls FNavigationSystem::DiscardNavigationDataChunks
                    // In a commandlet we load streaming levels by batch, build navigation for those levels only, then load another batch of levels, build navigation for those levels, etc...
                    // This means that this function ASVONavigationData::OnNavigationDataGenerationFinished is called after navigation is built for each batch of levels
                    // and that also means that after the last batch of levels is processed, we would release the navigation data for each previous batch of levels
                    if ( !IsRunningCommandlet() )
                    {
                        // stale data that is left in the level
                        if ( navigation_data_chunk != nullptr )
                        {
                            navigation_data_chunk->ReleaseNavigationData();
                            navigation_data_chunk->MarkPackageDirty();
                            level->NavDataChunks.Remove( navigation_data_chunk );
                        }
                    }
                }
            }

            // force navmesh drawing update
            RequestDrawingUpdate( /*bForce=*/true );
#endif // WITH_EDITOR

            UNavigationSystemV1 * NavSys = FNavigationSystem::GetCurrent< UNavigationSystemV1 >( world );
            if ( NavSys )
            {
                NavSys->OnNavigationGenerationFinished( *this );
            }

            DataInfos.Infos.Reset();

            for ( const auto & bounds_navigation_data : VolumeNavigationData )
            {
                auto & navigation_data_infos = DataInfos.Infos.AddDefaulted_GetRef();
                navigation_data_infos.VolumeLocation = bounds_navigation_data.GetVolumeBounds().GetCenter();
                navigation_data_infos.LayerCount = bounds_navigation_data.GetData().GetLayerCount();
                navigation_data_infos.bHasNavigationData = bounds_navigation_data.GetData().IsValid();
            }
        }
    }
}
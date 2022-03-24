#include "SVONavigationDataGenerator.h"

#include "SVONavigationData.h"

#include <GameFramework/PlayerController.h>
#include <NavigationSystem.h>

FSVOVolumeNavigationDataGenerator::FSVOVolumeNavigationDataGenerator( FSVONavigationDataGenerator & navigation_data_generator, const FBox & volume_bounds ) :
    ParentGenerator( navigation_data_generator ),
    BoundsNavigationData(),
    VolumeBounds( volume_bounds )
{
    NavDataConfig = navigation_data_generator.GetOwner()->GetConfig();
}

bool FSVOVolumeNavigationDataGenerator::DoWork()
{
    FSVOVolumeNavigationDataGenerationSettings generation_settings;
    generation_settings.GenerationSettings = ParentGenerator.GetGenerationSettings();
    generation_settings.World = ParentGenerator.GetWorld();
    generation_settings.VoxelExtent = NavDataConfig.AgentRadius * 2.0f;

    BoundsNavigationData.GenerateNavigationData( VolumeBounds, generation_settings );

    return true;
}

FSVONavigationDataGenerator::FSVONavigationDataGenerator( ASVONavigationData & navigation_data ) :
    NavigationData( navigation_data ),
    MaximumGeneratorTaskCount( 2 ),
    IsInitialized( false )
{
}

void FSVONavigationDataGenerator::Init()
{
    GenerationSettings = NavigationData.GenerationSettings;

    UpdateNavigationBounds();

    ///** setup maximum number of active tile generator*/
    const int32 worker_threads_count = FTaskGraphInterface::Get().GetNumWorkerThreads();
    MaximumGeneratorTaskCount = FMath::Min( FMath::Max( worker_threads_count * 2, 1 ), NavigationData.MaxSimultaneousBoxGenerationJobsCount );
    UE_LOG( LogNavigation, Log, TEXT( "Using max of %d workers to build SVO navigation." ), MaximumGeneratorTaskCount );

    //IsInitialized = true;

    //// recreate navmesh if no data was loaded, or when loaded data doesn't match current grid layout
    //bool must_create_navigation_data = true;
    //const bool is_static_navigation_data = IsStaticNavigationData( NavigationData );

    //if ( is_static_navigation_data )
    //{
    //    must_create_navigation_data = false;
    //}
    //else
    //{
    //    // :TODO: ?
    //};

    //if ( must_create_navigation_data )
    //{
    //    // :TODO:
    //    //ConstructSVOData();
    //    MarkNavBoundsDirty();
    //}
}

bool FSVONavigationDataGenerator::RebuildAll()
{
    NavigationData.UpdateNavVersion();

    UpdateNavigationBounds();

    TArray< FNavigationDirtyArea > dirty_areas;
    dirty_areas.Reserve( RegisteredNavigationBounds.Num() );

    for ( const auto & registered_navigation_bounds : RegisteredNavigationBounds )
    {
        dirty_areas.Emplace( FNavigationDirtyArea( registered_navigation_bounds, 0 ) );
    }

    RebuildDirtyAreas( dirty_areas );

    NavigationData.RequestDrawingUpdate();
    return true;
}

void FSVONavigationDataGenerator::EnsureBuildCompletion()
{
    const bool had_tasks = GetNumRemaningBuildTasks() > 0;

    do
    {
        const int32 tasks_to_process_count = MaximumGeneratorTaskCount - RunningBoundsDataGenerationElements.Num();
        ProcessAsyncTasks( tasks_to_process_count );

        // Block until tasks are finished
        for ( auto & element : RunningBoundsDataGenerationElements )
        {
            element.AsyncTask->EnsureCompletion();
        }
    } while ( GetNumRemaningBuildTasks() > 0 );

    if ( had_tasks )
    {
        NavigationData.RequestDrawingUpdate();
    }
}

void FSVONavigationDataGenerator::CancelBuild()
{
    PendingBoundsDataGenerationElements.Empty();

    for ( auto & element : RunningBoundsDataGenerationElements )
    {
        if ( element.AsyncTask )
        {
            element.AsyncTask->EnsureCompletion();
            delete element.AsyncTask;
            element.AsyncTask = nullptr;
        }
    }

    RunningBoundsDataGenerationElements.Empty();
}

void FSVONavigationDataGenerator::TickAsyncBuild( float delta_seconds )
{
    const UNavigationSystemV1 * navigation_system = FNavigationSystem::GetCurrent< UNavigationSystemV1 >( GetWorld() );
    if ( !ensureMsgf( navigation_system != nullptr, TEXT( "FRecastNavMeshGenerator can't found valid navigation system: Owner=[%s] World=[%s]" ), *GetFullNameSafe( GetOwner() ), *GetFullNameSafe( GetWorld() ) ) )
    {
        return;
    }

    const int32 running_tasks_count = navigation_system->GetNumRunningBuildTasks();

    const int32 tasks_to_submit_count = MaximumGeneratorTaskCount - running_tasks_count;

    const auto finished_boxes = ProcessAsyncTasks( tasks_to_submit_count );

    if ( finished_boxes.Num() > 0 )
    {
        NavigationData.OnNavigationDataUpdatedInBounds( finished_boxes );
        NavigationData.RequestDrawingUpdate();
    }
}

void FSVONavigationDataGenerator::OnNavigationBoundsChanged()
{
    UpdateNavigationBounds();
}

void FSVONavigationDataGenerator::RebuildDirtyAreas( const TArray< FNavigationDirtyArea > & dirty_areas )
{
    // The dirty areas are not always in the navigation bounds. If we move a static mesh outside of the navigation bounds, that function is called nonetheless
    // So let's first keep only the areas which are in the known navigation bounds
    for ( const auto & dirty_area : dirty_areas )
    {
        const auto matching_bounds = RegisteredNavigationBounds.FilterByPredicate( [ &dirty_area ]( const FBox & box ) {
            return box == dirty_area.Bounds || box.IsInside( dirty_area.Bounds ) || box.Intersect( dirty_area.Bounds );
        } );

        for ( const auto & matching_bounds_element : matching_bounds )
        {
            // Don't add another pending generation if one is already there for the navigation bounds the dirty area is in
            if ( PendingBoundsDataGenerationElements.FindByPredicate( [ &matching_bounds_element ]( const FPendingBoundsDataGenerationElement & pending_element ) {
                     return pending_element.VolumeBounds == matching_bounds_element;
                 } ) == nullptr )
            {
                FPendingBoundsDataGenerationElement pending_box_element;
                pending_box_element.VolumeBounds = matching_bounds_element;
                PendingBoundsDataGenerationElements.Emplace( pending_box_element );

                NavigationData.RemoveDataInBounds( matching_bounds_element );
            }
        }
    }

    // Sort tiles by proximity to players
    if ( PendingBoundsDataGenerationElements.Num() > 0 )
    {
        SortPendingBounds();
    }
}

bool FSVONavigationDataGenerator::IsBuildInProgressCheckDirty() const
{
    return RunningBoundsDataGenerationElements.Num() || PendingBoundsDataGenerationElements.Num();
}

int32 FSVONavigationDataGenerator::GetNumRemaningBuildTasks() const
{
    return RunningBoundsDataGenerationElements.Num() + PendingBoundsDataGenerationElements.Num();
}

int32 FSVONavigationDataGenerator::GetNumRunningBuildTasks() const
{
    return RunningBoundsDataGenerationElements.Num();
}

void FSVONavigationDataGenerator::GetSeedLocations( TArray< FVector2D > & seed_locations, UWorld & world ) const
{
    // Collect players positions
    for ( FConstPlayerControllerIterator player_iterator = world.GetPlayerControllerIterator(); player_iterator; ++player_iterator )
    {
        if ( APlayerController * player_controller = player_iterator->Get() )
        {
            if ( auto * pawn = player_controller->GetPawn() )
            {
                const FVector2D seed_location( pawn->GetActorLocation() );
                seed_locations.Add( seed_location );
            }
        }
    }
}

void FSVONavigationDataGenerator::SortPendingBounds()
{
    if ( UWorld * current_world = GetWorld() )
    {
        TArray< FVector2D > seed_locations;
        GetSeedLocations( seed_locations, *current_world );

        if ( seed_locations.Num() == 0 )
        {
            seed_locations.Add( FVector2D( TotalNavigationBounds.GetCenter() ) );
        }

        if ( seed_locations.Num() > 0 )
        {
            for ( auto & element : PendingBoundsDataGenerationElements )
            {
                FVector2D tile_center_2d = FVector2D( element.VolumeBounds.GetCenter() );
                for ( const auto & seed_location : seed_locations )
                {
                    element.SeedDistance = FMath::Min( element.SeedDistance, FVector2D::DistSquared( tile_center_2d, seed_location ) );
                }
            }

            PendingBoundsDataGenerationElements.Sort();
        }
    }
}

void FSVONavigationDataGenerator::UpdateNavigationBounds()
{
    if ( const UNavigationSystemV1 * navigation_system = FNavigationSystem::GetCurrent< UNavigationSystemV1 >( GetWorld() ) )
    {
        if ( !navigation_system->ShouldGenerateNavigationEverywhere() )
        {
            FBox bounds_sum( ForceInit );
            {
                TArray< FBox > supported_navigation_bounds;
                navigation_system->GetNavigationBoundsForNavData( NavigationData, supported_navigation_bounds );

                RegisteredNavigationBounds.Reset( supported_navigation_bounds.Num() );

                for ( const auto & box : supported_navigation_bounds )
                {
                    RegisteredNavigationBounds.Add( box );
                    bounds_sum += box;
                }

                // Remove the existing navigation bounds which don't match the new navigation bounds
                // :TODO: Avoid copying data
                auto existing_navigation_bounds = NavigationData.GetVolumeNavigationData();

                for ( const auto & existing_bounds : existing_navigation_bounds )
                {
                    if ( RegisteredNavigationBounds.FindByPredicate( [ &existing_bounds ]( const FBox & registered_bounds ) {
                             return registered_bounds == existing_bounds.GetVolumeBounds();
                         } ) == nullptr )
                    {
                        NavigationData.RemoveDataInBounds( existing_bounds.GetVolumeBounds() );
                    }
                }
            }
            TotalNavigationBounds = bounds_sum;
        }
        else
        {
            RegisteredNavigationBounds.Reset( 1 );
            TotalNavigationBounds = navigation_system->GetWorldBounds();
            if ( !TotalNavigationBounds.IsValid )
            {
                RegisteredNavigationBounds.Add( TotalNavigationBounds );
            }
        }
    }
    else
    {
        TotalNavigationBounds = FBox( ForceInit );
    }
}

TArray< FBox > FSVONavigationDataGenerator::ProcessAsyncTasks( const int32 task_to_process_count )
{
    const bool has_tasks_at_start = GetNumRemaningBuildTasks() > 0;

    int32 processed_tasks_count = 0;
    // Submit pending tile elements
    for ( int32 element_index = PendingBoundsDataGenerationElements.Num() - 1; element_index >= 0 && processed_tasks_count < task_to_process_count; element_index-- )
    {
        FPendingBoundsDataGenerationElement & PendingElement = PendingBoundsDataGenerationElements[ element_index ];
        FRunningBoundsDataGenerationElement running_element( PendingElement.VolumeBounds );

        if ( RunningBoundsDataGenerationElements.Contains( running_element ) )
        {
            continue;
        }

        TUniquePtr< FSVOBoxGeneratorTask > task = MakeUnique< FSVOBoxGeneratorTask >( CreateBoxNavigationGenerator( PendingElement.VolumeBounds ) );

        running_element.AsyncTask = task.Release();

        running_element.AsyncTask->StartBackgroundTask();

        RunningBoundsDataGenerationElements.Add( running_element );

        PendingBoundsDataGenerationElements.RemoveAt( element_index, 1, /*bAllowShrinking=*/false );
        processed_tasks_count++;
    }

    if ( processed_tasks_count > 0 && PendingBoundsDataGenerationElements.Num() == 0 )
    {
        PendingBoundsDataGenerationElements.Empty( 64 );
    }

    TArray< FBox > finished_boxes;

    for ( int32 index = RunningBoundsDataGenerationElements.Num() - 1; index >= 0; --index )
    {
        //QUICK_SCOPE_CYCLE_COUNTER( STAT_RecastNavMeshGenerator_ProcessTileTasks_FinishedTasks );

        FRunningBoundsDataGenerationElement & element = RunningBoundsDataGenerationElements[ index ];
        check( element.AsyncTask != nullptr );

        if ( !element.AsyncTask->IsDone() )
        {
            continue;
        }

        if ( element.ShouldDiscard )
        {
            continue;
        }

        auto & box_generator = *element.AsyncTask->GetTask().BoxNavigationDataGenerator;

        NavigationData.AddVolumeNavigationData( box_generator.GetBoundsNavigationData() );

        finished_boxes.Emplace( MoveTemp( element.VolumeBounds ) );

        delete element.AsyncTask;
        element.AsyncTask = nullptr;
        RunningBoundsDataGenerationElements.RemoveAtSwap( index, 1, false );
    }

    const bool has_tasks_at_end = GetNumRemaningBuildTasks() > 0;
    if ( has_tasks_at_start && !has_tasks_at_end )
    {
        //QUICK_SCOPE_CYCLE_COUNTER( STAT_RecastNavMeshGenerator_OnNavMeshGenerationFinished );
        NavigationData.OnNavigationDataGenerationFinished();
    }

    return finished_boxes;
}

TSharedRef< FSVOVolumeNavigationDataGenerator > FSVONavigationDataGenerator::CreateBoxNavigationGenerator( const FBox & box )
{
    //SCOPE_CYCLE_COUNTER(STAT_SVONavigation_CreateBoxNavigationGenerator);

    TSharedRef< FSVOVolumeNavigationDataGenerator > box_navigation_data_generator = MakeShareable( new FSVOVolumeNavigationDataGenerator( *this, box ) );
    return box_navigation_data_generator;
}

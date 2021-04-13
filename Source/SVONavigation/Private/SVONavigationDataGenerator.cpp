#include "SVONavigationDataGenerator.h"

#include "SVONavigationData.h"

#include <NavigationSystem.h>

FSVOBoxNavigationDataGenerator::FSVOBoxNavigationDataGenerator( FSVONavigationDataGenerator & navigation_data_generator, const FBox & volume_bounds ) :
    ParentGenerator( navigation_data_generator ),
    VolumeBounds( volume_bounds )
{
    NavDataConfig = navigation_data_generator.GetOwner()->GetConfig();
}

bool FSVOBoxNavigationDataGenerator::DoWork()
{
    FSVOBoundsNavigationDataGenerationSettings generation_settings;
    generation_settings.GenerationSettings = ParentGenerator.GetGenerationSettings();
    generation_settings.World = ParentGenerator.GetWorld();
    generation_settings.VoxelSize = FMath::Max( NavDataConfig.AgentHeight, NavDataConfig.AgentRadius );

    BoundsNavigationData.GenerateNavigationData( VolumeBounds, generation_settings );

    return true;
}

FSVONavigationDataGenerator::FSVONavigationDataGenerator( ASVONavigationData & navigation_data ) :
    NavigationData( navigation_data ),
    IsInitialized( false )
{
}

FSVONavigationDataGenerator::~FSVONavigationDataGenerator()
{
}

void FSVONavigationDataGenerator::Init()
{
    GenerationSettings = NavigationData.GenerationSettings;

    UpdateNavigationBounds();

    ///** setup maximum number of active tile generator*/
    //const int32 NumberOfWorkerThreads = FTaskGraphInterface::Get().GetNumWorkerThreads();
    //MaxBoxGeneratorTasks = FMath::Min( FMath::Max( NumberOfWorkerThreads * 2, 1 ), NavigationData.MaxSimultaneousBoxGenerationJobsCount );
    //UE_LOG( LogNavigation, Log, TEXT( "Using max of %d workers to build SVO navigation." ), MaxBoxGeneratorTasks );

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
    NavigationData.RequestDrawingUpdate();
    return true;
}

void FSVONavigationDataGenerator::EnsureBuildCompletion()
{
    const bool had_taks = GetNumRemaningBuildTasks() > 0;

    do
    {
        const int32 tasks_to_process_count = MaxBoxGeneratorTasks - RunningDirtyBoxes.Num();
        ProcessAsyncTasks( tasks_to_process_count );

        // Block until tasks are finished
        for ( auto & element : RunningDirtyBoxes )
        {
            element.AsyncTask->EnsureCompletion();
        }
    } while ( GetNumRemaningBuildTasks() > 0 );

    if ( had_taks )
    {
        NavigationData.RequestDrawingUpdate();
    }
}

void FSVONavigationDataGenerator::CancelBuild()
{
    PendingDirtyBoxes.Empty();

    for ( auto & element : RunningDirtyBoxes )
    {
        if ( element.AsyncTask )
        {
            element.AsyncTask->EnsureCompletion();
            delete element.AsyncTask;
            element.AsyncTask = nullptr;
        }
    }

    RunningDirtyBoxes.Empty();
}

void FSVONavigationDataGenerator::TickAsyncBuild( float delta_seconds )
{
    const UNavigationSystemV1 * navigation_system = FNavigationSystem::GetCurrent< UNavigationSystemV1 >( GetWorld() );
    if ( !ensureMsgf( navigation_system != nullptr, TEXT( "FRecastNavMeshGenerator can't found valid navigation system: Owner=[%s] World=[%s]" ), *GetFullNameSafe( GetOwner() ), *GetFullNameSafe( GetWorld() ) ) )
    {
        return;
    }

    const int32 NumRunningTasks = navigation_system->GetNumRunningBuildTasks();

    const int32 NumTasksToSubmit = MaxBoxGeneratorTasks - NumRunningTasks;

    const auto finished_boxes = ProcessAsyncTasks( NumTasksToSubmit );

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
    TSet< FPendingBoxElement > dirty_bounds_elements;
    TSet< FBox > bounds_to_delete;

    for ( const auto & dirty_area : dirty_areas )
    {
        auto * existing_bounds = RegisteredBounds.FindByPredicate( [ &dirty_area ]( const FBox & box ) {
            return box == dirty_area.Bounds;
        } );

        if ( existing_bounds == nullptr )
        {
            bounds_to_delete.Add( dirty_area.Bounds );
            continue;
        }

        FPendingBoxElement pending_box_element;
        pending_box_element.VolumeBounds = dirty_area.Bounds;
        dirty_bounds_elements.Emplace( pending_box_element );
    }

    for ( const auto & bound_to_delete : bounds_to_delete )
    {
        for ( auto index = NavigationData.NavigationBoundsData.Num() - 1; index >= 0; --index )
        {
            NavigationData.NavigationBoundsData.Remove( bound_to_delete );
        }
    }

    PendingDirtyBoxes.Reserve( PendingDirtyBoxes.Num() + dirty_bounds_elements.Num() );
    for ( const auto & dirty_box : dirty_bounds_elements )
    {
        PendingDirtyBoxes.Emplace( dirty_box );
    }

    //// Sort tiles by proximity to players
    //if ( NumTilesMarked > 0 )
    //{
    //    SortPendingBuildTiles();
    //}
}

bool FSVONavigationDataGenerator::IsBuildInProgressCheckDirty() const
{
    return RunningDirtyBoxes.Num() || PendingDirtyBoxes.Num();
}

int32 FSVONavigationDataGenerator::GetNumRemaningBuildTasks() const
{
    return RunningDirtyBoxes.Num() + PendingDirtyBoxes.Num();
}

int32 FSVONavigationDataGenerator::GetNumRunningBuildTasks() const
{
    return RunningDirtyBoxes.Num();
}

uint32 FSVONavigationDataGenerator::LogMemUsed() const
{
    return 0;
}

void FSVONavigationDataGenerator::UpdateNavigationBounds()
{
    if ( const UNavigationSystemV1 * navigation_system = FNavigationSystem::GetCurrent< UNavigationSystemV1 >( GetWorld() ) )
    {
        if ( !navigation_system->ShouldGenerateNavigationEverywhere() )
        {
            FBox BoundsSum( ForceInit );
            {
                TArray< FBox > SupportedBounds;
                navigation_system->GetNavigationBoundsForNavData( NavigationData, SupportedBounds );
                RegisteredBounds.Reset( SupportedBounds.Num() );

                for ( const FBox & Box : SupportedBounds )
                {
                    RegisteredBounds.Add( Box );
                    BoundsSum += Box;
                }
            }
            TotalNavBounds = BoundsSum;
        }
        else
        {
            RegisteredBounds.Reset( 1 );
            TotalNavBounds = navigation_system->GetWorldBounds();
            if ( !TotalNavBounds.IsValid )
            {
                RegisteredBounds.Add( TotalNavBounds );
            }
        }
    }
    else
    {
        TotalNavBounds = FBox( ForceInit );
    }
}

TArray< FBox > FSVONavigationDataGenerator::ProcessAsyncTasks( int32 task_to_process_count )
{
    int32 processed_tasks_count = 0;
    // Submit pending tile elements
    for ( int32 element_index = PendingDirtyBoxes.Num() - 1; element_index >= 0 && processed_tasks_count < task_to_process_count; element_index-- )
    {
        FPendingBoxElement & PendingElement = PendingDirtyBoxes[ element_index ];
        FRunningBoxElement running_element( PendingElement.VolumeBounds );

        if ( RunningDirtyBoxes.Contains( running_element ) )
        {
            continue;
        }

        TUniquePtr< FSVOBoxGeneratorTask > task = MakeUnique< FSVOBoxGeneratorTask >( CreateBoxNavigationGenerator( PendingElement.VolumeBounds ) );

        running_element.AsyncTask = task.Release();

        running_element.AsyncTask->StartBackgroundTask();

        RunningDirtyBoxes.Add( running_element );

        PendingDirtyBoxes.RemoveAt( element_index, 1, /*bAllowShrinking=*/false );
        processed_tasks_count++;
    }

    if ( processed_tasks_count > 0 && PendingDirtyBoxes.Num() == 0 )
    {
        PendingDirtyBoxes.Empty( 64 );
    }

    TArray< FBox > finished_boxes;

    for ( int32 index = RunningDirtyBoxes.Num() - 1; index >= 0; --index )
    {
        //QUICK_SCOPE_CYCLE_COUNTER( STAT_RecastNavMeshGenerator_ProcessTileTasks_FinishedTasks );

        FRunningBoxElement & element = RunningDirtyBoxes[ index ];
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

        const auto & bounds_navigation_data = box_generator.GetBoundsNavigationData();
        NavigationData.NavigationBoundsData.Add( bounds_navigation_data.GetVolumeBounds(), bounds_navigation_data );

        finished_boxes.Emplace( MoveTemp( element.VolumeBounds ) );

        delete element.AsyncTask;
        element.AsyncTask = nullptr;
        RunningDirtyBoxes.RemoveAtSwap( index, 1, false );
    }

    return finished_boxes;
}

TSharedRef< FSVOBoxNavigationDataGenerator > FSVONavigationDataGenerator::CreateBoxNavigationGenerator( const FBox & box )
{
    //SCOPE_CYCLE_COUNTER(STAT_SVONavigation_CreateBoxNavigationGenerator);

    TSharedRef< FSVOBoxNavigationDataGenerator > box_navigation_data_generator = MakeShareable( new FSVOBoxNavigationDataGenerator( *this, box ) );
    return box_navigation_data_generator;
}

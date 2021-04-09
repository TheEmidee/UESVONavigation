#include "SVONavigationSystem.h"

#include "SVONavigationData.h"

#include <EngineUtils.h>

void USVONavigationSystem::Initialize( FSubsystemCollectionBase & collection )
{
    Super::Initialize( collection );

    TickDelegate = FTickerDelegate::CreateUObject( this, &USVONavigationSystem::Tick );
    TickDelegateHandle = FTicker::GetCoreTicker().AddTicker( TickDelegate );

    RegisterWorldDelegates();

#if WITH_EDITOR
    if ( GIsEditor )
    {
        GEngine->OnActorMoved().AddUObject( this, &USVONavigationSystem::OnActorMoved );
    }
#endif
}

void USVONavigationSystem::Deinitialize()
{
    Super::Deinitialize();

    FTicker::GetCoreTicker().RemoveTicker( TickDelegateHandle );

    UnRegisterWorldDelegates();
}

UWorld * USVONavigationSystem::GetWorld() const
{
    return World.Get();
}

void USVONavigationSystem::OnNavigationVolumeAdded( const ASVONavigationVolume & volume )
{
    auto * volume_world = volume.GetWorld();

    if ( volume_world != World )
    {
        return;
    }

    FSVONavigationBoundsUpdateRequest UpdateRequest;
    UpdateRequest.NavBounds.UniqueID = volume.GetUniqueID();
    UpdateRequest.NavBounds.AreaBox = volume.GetComponentsBoundingBox( true );
    UpdateRequest.NavBounds.Level = volume.GetLevel();

    UpdateRequest.UpdateRequest = FSVONavigationBoundsUpdateRequest::Type::Added;
    AddNavigationVolumeUpdateRequest( UpdateRequest );
}

void USVONavigationSystem::OnNavigationVolumeRemoved( const ASVONavigationVolume & volume )
{
    auto * volume_world = volume.GetWorld();

    if ( volume_world != World )
    {
        return;
    }

    FSVONavigationBoundsUpdateRequest UpdateRequest;
    UpdateRequest.NavBounds.UniqueID = volume.GetUniqueID();
    UpdateRequest.NavBounds.AreaBox = volume.GetComponentsBoundingBox( true );
    UpdateRequest.NavBounds.Level = volume.GetLevel();

    UpdateRequest.UpdateRequest = FSVONavigationBoundsUpdateRequest::Type::Removed;
    AddNavigationVolumeUpdateRequest( UpdateRequest );
}

void USVONavigationSystem::OnNavigationVolumeUpdated( const ASVONavigationVolume & volume )
{
    auto * volume_world = volume.GetWorld();

    if ( volume_world != World )
    {
        return;
    }

    FSVONavigationBoundsUpdateRequest UpdateRequest;
    UpdateRequest.NavBounds.UniqueID = volume.GetUniqueID();
    UpdateRequest.NavBounds.AreaBox = volume.GetComponentsBoundingBox( true );
    UpdateRequest.NavBounds.Level = volume.GetLevel();

    UpdateRequest.UpdateRequest = FSVONavigationBoundsUpdateRequest::Type::Updated;
    AddNavigationVolumeUpdateRequest( UpdateRequest );
}

bool USVONavigationSystem::Tick( const float /*delta_seconds*/ )
{
    if ( PendingNavigationVolumeUpdateRequests.Num() > 0 )
    {
        PerformNavigationVolumesUpdate( PendingNavigationVolumeUpdateRequests );
        PendingNavigationVolumeUpdateRequests.Reset();
    }

    // In multithreaded configuration we can process async pathfinding queries
    // in dedicated task while dispatching completed queries results on the main thread.
    // The created task can start and append new result right away so we transfer
    // completed queries before to keep the list safe.
    TArray< FSVOAsyncPathFindingQuery > AsyncPathFindingCompletedQueriesToDispatch;
    Swap( AsyncPathFindingCompletedQueriesToDispatch, AsyncPathFindingCompletedQueries );

    // Trigger the async pathfinding queries (new ones and those that may have been postponed from last frame)
    if ( AsyncPathFindingQueries.Num() > 0 )
    {
        TriggerAsyncQueries( AsyncPathFindingQueries );
        AsyncPathFindingQueries.Reset();
    }

    // Dispatch async pathfinding queries results from last frame
    DispatchAsyncQueriesResults( AsyncPathFindingCompletedQueriesToDispatch );

    return true;
}

void USVONavigationSystem::UpdateAllNavigationVolumes()
{
    for ( TActorIterator< ASVONavigationVolume > iterator( GetWorld() ); iterator; ++iterator )
    {
        OnNavigationVolumeUpdated( **iterator );
    }
}

FSVOPathFindingResult USVONavigationSystem::FindPathSync( FSVOPathFindingQuery path_finding_query )
{
    if ( !path_finding_query.NavigationData.IsValid() )
    {
        path_finding_query.NavigationData = GetDefaultNavigationDataInstance( false );
    }

    FSVOPathFindingResult result( ENavigationQueryResult::Error );
    if ( path_finding_query.NavigationData.IsValid() )
    {
        result = path_finding_query.NavigationData->FindPath( path_finding_query );
    }

    return result;
}

uint32 USVONavigationSystem::FindPathAsync( FSVOPathFindingQuery path_finding_query, const FSVONavigationPathQueryDelegate & result_delegate )
{
    if ( !path_finding_query.NavigationData.IsValid() )
    {
        path_finding_query.NavigationData = GetDefaultNavigationDataInstance( false );
    }

    if ( path_finding_query.NavigationData.IsValid() )
    {
        FSVOAsyncPathFindingQuery async_query( path_finding_query, result_delegate );

        if ( async_query.QueryID != INVALID_NAVQUERYID )
        {
            AddAsyncQuery( async_query );
        }

        return async_query.QueryID;
    }

    return INVALID_NAVQUERYID;
}

#if WITH_EDITOR
void USVONavigationSystem::OnActorMoved( AActor * actor )
{
    if ( auto * volume = Cast< ASVONavigationVolume >( actor ) )
    {
        OnNavigationVolumeUpdated( *volume );
    }
    else
    {
        UpdateNavigationVolumeAroundActor( actor );
    }
}
#endif

void USVONavigationSystem::RegisterWorldDelegates()
{
    PostLoadMapWithWorldDelegateHandle = FCoreUObjectDelegates::PostLoadMapWithWorld.AddUObject( this, &USVONavigationSystem::OnPostLoadMap );
    OnPostWorldInitializationDelegateHandle = FWorldDelegates::OnPostWorldInitialization.AddUObject( this, &USVONavigationSystem::OnPostWorldInitialized );
    OnPostWorldCleanupDelegateHandle = FWorldDelegates::OnPostWorldCleanup.AddUObject( this, &USVONavigationSystem::OnPostWorldCleanup );
    OnLevelAddedToWorldDelegateHandle = FWorldDelegates::LevelAddedToWorld.AddUObject( this, &USVONavigationSystem::OnLevelAddedToWorld );
    OnLevelRemovedFromWorldDelegateHandle = FWorldDelegates::LevelRemovedFromWorld.AddUObject( this, &USVONavigationSystem::OnLevelRemovedFromWorld );
    OnWorldPostActorTickDelegateHandle = FWorldDelegates::OnWorldPostActorTick.AddUObject( this, &USVONavigationSystem::OnWorldPostActorTick );
}

void USVONavigationSystem::UnRegisterWorldDelegates()
{
    FCoreUObjectDelegates::PostLoadMapWithWorld.Remove( PostLoadMapWithWorldDelegateHandle );
    FWorldDelegates::OnPostWorldInitialization.Remove( OnPostWorldInitializationDelegateHandle );
    FWorldDelegates::LevelAddedToWorld.Remove( OnLevelAddedToWorldDelegateHandle );
    FWorldDelegates::LevelRemovedFromWorld.Remove( OnLevelRemovedFromWorldDelegateHandle );
    FWorldDelegates::OnWorldPostActorTick.Remove( OnWorldPostActorTickDelegateHandle );
}

void USVONavigationSystem::OnPostWorldInitialized( UWorld * world, const UWorld::InitializationValues /*initialization_values*/ )
{
    NavigationData.Reset();
    PendingNavigationVolumeUpdateRequests.Empty();

    World = world;
    GatherExistingNavigationData();
}

void USVONavigationSystem::OnPostWorldCleanup( UWorld * world, bool session_ended, bool cleanup_resources )
{
    /*if ( NavigationData.IsValid() && NavigationData->GetWorld() != World )
    {
        NavigationData.Reset();
    }*/
}

void USVONavigationSystem::OnLevelAddedToWorld( ULevel * /* level */, UWorld * world )
{
    check( World.IsValid() )
}

void USVONavigationSystem::OnLevelRemovedFromWorld( ULevel * level, UWorld * world )
{
    check( World.IsValid() )
}

void USVONavigationSystem::OnWorldPostActorTick( UWorld * world, ELevelTick tick_type, float delta_time )
{
    PostponeAsyncQueries();
}

void USVONavigationSystem::OnPostLoadMap( UWorld * world )
{
    World = world;
    ASVONavigationData * navigation_data = GetDefaultNavigationDataInstance( false );

    if ( navigation_data == nullptr && IsThereAnywhereToBuildNavigation() )
    {
        navigation_data = GetDefaultNavigationDataInstance( true );
    }
}

ASVONavigationData * USVONavigationSystem::CreateNavigationData() const
{
    if ( World != nullptr )
    {
        return World->SpawnActor< ASVONavigationData >( ASVONavigationData::StaticClass() );
    }

    return nullptr;
}

void USVONavigationSystem::GatherExistingNavigationData()
{
    check( World != nullptr );

    for ( TActorIterator< ASVONavigationData > iterator( World.Get() ); iterator; ++iterator )
    {
        ASVONavigationData * navigation_data = ( *iterator );
        if ( navigation_data != nullptr && !navigation_data->IsPendingKill() )
        {
            NavigationData = navigation_data;
            return;
        }
    }
}

void USVONavigationSystem::SpawnMissingNavigationData()
{
    if ( NavigationData == nullptr )
    {
        NavigationData = CreateNavigationData();
    }
}

ASVONavigationData * USVONavigationSystem::GetDefaultNavigationDataInstance( bool it_creates_if_missing )
{
    if ( NavigationData == nullptr || NavigationData->IsPendingKill() )
    {
        if ( it_creates_if_missing )
        {
            NavigationData = CreateNavigationData();
        }
    }

    return NavigationData.Get();
}

void USVONavigationSystem::PerformNavigationVolumesUpdate( const TArray< FSVONavigationBoundsUpdateRequest > & update_requests )
{
    SpawnMissingNavigationData();

    if ( !NavigationData.IsValid() )
    {
        return;
    }

    for ( const auto & update_request : update_requests )
    {
        FSetElementId ExistingElementId = RegisteredNavigationBounds.FindId( update_request.NavBounds );

        switch ( update_request.UpdateRequest )
        {
            case FSVONavigationBoundsUpdateRequest::Type::Removed:
            {
                if ( ExistingElementId.IsValidId() )
                {
                    RegisteredNavigationBounds.Remove( ExistingElementId );
                    NavigationData->RemoveNavigationBounds( update_request.NavBounds );
                }
            }
            break;

            case FSVONavigationBoundsUpdateRequest::Type::Added:
            case FSVONavigationBoundsUpdateRequest::Type::Updated:
            {
                if ( ExistingElementId.IsValidId() )
                {
                    // always assign new bounds data, it may have different properties (like supported agents)
                    RegisteredNavigationBounds[ ExistingElementId ] = update_request.NavBounds;
                    NavigationData->UpdateNavigationBounds( update_request.NavBounds );
                }
                else
                {
                    RegisteredNavigationBounds.Emplace( update_request.NavBounds );
                    NavigationData->AddNavigationBounds( update_request.NavBounds );
                }
            }
            break;
        }
    }
}

void USVONavigationSystem::AddNavigationVolumeUpdateRequest( const FSVONavigationBoundsUpdateRequest & update_request )
{
    const auto existing_id = PendingNavigationVolumeUpdateRequests.IndexOfByPredicate( [ & ]( const FSVONavigationBoundsUpdateRequest & Element ) {
        return update_request.NavBounds.UniqueID == Element.NavBounds.UniqueID;
    } );

    if ( existing_id != INDEX_NONE )
    {
        // catch the case where the bounds was removed and immediately re-added with the same bounds as before
        // in that case, we can cancel any update at all
        auto it_can_cancel_update = false;
        if ( PendingNavigationVolumeUpdateRequests[ existing_id ].UpdateRequest == FSVONavigationBoundsUpdateRequest::Type::Removed && update_request.UpdateRequest == FSVONavigationBoundsUpdateRequest::Type::Added )
        {
            for ( TSet< FSVONavigationBounds >::TConstIterator It( RegisteredNavigationBounds ); It; ++It )
            {
                if ( *It == update_request.NavBounds )
                {
                    it_can_cancel_update = true;
                    break;
                }
            }
        }
        if ( it_can_cancel_update )
        {
            PendingNavigationVolumeUpdateRequests.RemoveAt( existing_id );
        }
        else
        {
            // Overwrite any previous updates
            PendingNavigationVolumeUpdateRequests[ existing_id ] = update_request;
        }
    }
    else
    {
        PendingNavigationVolumeUpdateRequests.Add( update_request );
    }
}

bool USVONavigationSystem::IsThereAnywhereToBuildNavigation() const
{
    for ( const auto & bounds : RegisteredNavigationBounds )
    {
        if ( bounds.AreaBox.IsValid )
        {
            return true;
        }
    }

    for ( TActorIterator< ASVONavigationVolume > It( GetWorld() ); It; ++It )
    {
        auto const * const volume = ( *It );
        if ( volume != nullptr && !volume->IsPendingKill() )
        {
            return true;
        }
    }

    return false;
}

void USVONavigationSystem::UpdateNavigationVolumeAroundActor( AActor * actor )
{
    if ( actor == nullptr )
    {
        return;
    }

    const auto actor_bounds = actor->GetComponentsBoundingBox();

    if ( auto * navigation_data = GetDefaultNavigationDataInstance( true ) )
    {
        for ( const auto & navigation_bounds : RegisteredNavigationBounds )
        {
            if ( navigation_bounds.AreaBox.Intersect( actor_bounds ) || actor_bounds.IsInside( navigation_bounds.AreaBox ) )
            {
                navigation_data->UpdateNavigationBounds( navigation_bounds );
            }
        }
    }
}

void USVONavigationSystem::AddAsyncQuery( const FSVOAsyncPathFindingQuery & async_query )
{
    check( IsInGameThread() );
    AsyncPathFindingQueries.Add( async_query );
}

void USVONavigationSystem::TriggerAsyncQueries( TArray< FSVOAsyncPathFindingQuery > & queries )
{
    DECLARE_CYCLE_STAT( TEXT( "FSimpleDelegateGraphTask.USVONavigationSystem batched async queries" ),
        STAT_FSimpleDelegateGraphTask_USVONavigationSystemBatchedAsyncQueries,
        STATGROUP_TaskGraphTasks );

    AsyncPathFindingTask = FSimpleDelegateGraphTask::CreateAndDispatchWhenReady(
        FSimpleDelegateGraphTask::FDelegate::CreateUObject( this, &USVONavigationSystem::PerformAsyncQueries, queries ),
        GET_STATID( STAT_FSimpleDelegateGraphTask_USVONavigationSystemBatchedAsyncQueries ) );
}

void USVONavigationSystem::DispatchAsyncQueriesResults( const TArray< FSVOAsyncPathFindingQuery > & queries )
{
    if ( AsyncPathFindingQueries.Num() > 0 )
    {
        for ( const auto & query : AsyncPathFindingQueries )
        {
            query.OnDoneDelegate.ExecuteIfBound( query.QueryID, query.Result.Result, query.Result.Path );
        }
    }
}

void USVONavigationSystem::PerformAsyncQueries( TArray< FSVOAsyncPathFindingQuery > path_finding_queries )
{
    if ( path_finding_queries.Num() == 0 )
    {
        return;
    }

    int32 NumProcessed = 0;
    for ( FSVOAsyncPathFindingQuery & query : path_finding_queries )
    {
        // @todo this is not necessarily the safest way to use UObjects outside of main thread.
        //	think about something else.
        if ( const auto * navigation_data = query.NavigationData.IsValid() ? query.NavigationData.Get() : GetDefaultNavigationDataInstance( false ) )
        {
            query.Result = navigation_data->FindPath( query );
        }
        else
        {
            query.Result.Result = ENavigationQueryResult::Error;
        }
        ++NumProcessed;

        // Check for abort request from the main tread
        if ( AbortAsyncQueriesRequested )
        {
            break;
        }
    }

    const int32 NumQueries = path_finding_queries.Num();
    const int32 NumPostponed = NumQueries - NumProcessed;

    // Queue remaining queries for next frame
    if ( AbortAsyncQueriesRequested )
    {
        AsyncPathFindingQueries.Append( path_finding_queries.GetData() + NumProcessed, NumPostponed );
    }

    // Append to list of completed queries to dispatch results in main thread
    AsyncPathFindingCompletedQueries.Append( path_finding_queries.GetData(), NumProcessed );

    UE_LOG( LogNavigation, Log, TEXT( "Async pathfinding queries: %d completed, %d postponed to next frame" ), NumProcessed, NumPostponed );
}

void USVONavigationSystem::PostponeAsyncQueries()
{
    if ( AsyncPathFindingTask.GetReference() && !AsyncPathFindingTask->IsComplete() )
    {
        AbortAsyncQueriesRequested = true;
        FTaskGraphInterface::Get().WaitUntilTaskCompletes( AsyncPathFindingTask, ENamedThreads::GameThread );
        AbortAsyncQueriesRequested = false;
    }
}

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

    return true;
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
}

void USVONavigationSystem::UnRegisterWorldDelegates()
{
    FCoreUObjectDelegates::PostLoadMapWithWorld.Remove( PostLoadMapWithWorldDelegateHandle );
    FWorldDelegates::OnPostWorldInitialization.Remove( OnPostWorldInitializationDelegateHandle );
    FWorldDelegates::LevelAddedToWorld.Remove( OnLevelAddedToWorldDelegateHandle );
    FWorldDelegates::LevelRemovedFromWorld.Remove( OnLevelRemovedFromWorldDelegateHandle );
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
    check( World != nullptr );
    return World->SpawnActor< ASVONavigationData >( ASVONavigationData::StaticClass() );
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

#include "SVOAITask_MoveTo.h"

#include "SVONavigationSystem.h"
#include "SVONavigationTypes.h"

#include <AIController.h>
#include <AIResources.h>
#include <Perception/AIPerceptionComponent.h>
#include <Runtime/Engine/Public/VisualLogger/VisualLogger.h>
#include <Runtime/GameplayTasks/Classes/GameplayTasksComponent.h>

USVOAITask_MoveTo::USVOAITask_MoveTo( const FObjectInitializer & object_initializer ) :
    Super( object_initializer )
{
    bIsPausable = true;
    MoveRequestID = FAIRequestID::InvalidRequest;

    MoveRequest.SetAcceptanceRadius( GET_AI_CONFIG_VAR( AcceptanceRadius ) );
    MoveRequest.SetReachTestIncludesAgentRadius( GET_AI_CONFIG_VAR( bFinishMoveOnGoalOverlap ) );
    MoveRequest.SetAllowPartialPath( GET_AI_CONFIG_VAR( bAcceptPartialPaths ) );
    MoveRequest.SetUsePathfinding( true );

    AddRequiredResource( UAIResource_Movement::StaticClass() );
    AddClaimedResource( UAIResource_Movement::StaticClass() );

    MoveResult = EPathFollowingResult::Invalid;
    UseContinuousTracking = false;
}

bool USVOAITask_MoveTo::WasMoveSuccessful() const
{
    return MoveResult == EPathFollowingResult::Success;
}

void USVOAITask_MoveTo::PerformMove()
{
    UPathFollowingComponent * path_following_component = GetPathFollowingComponent();
    if ( GetPathFollowingComponent() == nullptr )
    {
        FinishMoveTask( EPathFollowingResult::Invalid );
        return;
    }

    ResetObservers();
    ResetTimers();

    FNavPathSharedPtr followed_path;
    const FPathFollowingRequestResult ResultData = MoveTo( &followed_path, MoveRequest );

    switch ( ResultData.Code )
    {
        case EPathFollowingRequestResult::Failed:
            FinishMoveTask( EPathFollowingResult::Invalid );
            break;

        case EPathFollowingRequestResult::AlreadyAtGoal:
            MoveRequestID = ResultData.MoveId;
            OnRequestFinished( ResultData.MoveId, FPathFollowingResult( EPathFollowingResult::Success, FPathFollowingResultFlags::AlreadyAtGoal ) );
            break;

        case EPathFollowingRequestResult::RequestSuccessful:
            if ( UseAsyncPathFinding )
            {
            }
            else
            {
                MoveRequestID = ResultData.MoveId;
                PathFinishDelegateHandle = path_following_component->OnRequestFinished.AddUObject( this, &USVOAITask_MoveTo::OnRequestFinished );
                SetObservedPath( followed_path );

                if ( IsFinished() )
                {
                    UE_VLOG( GetGameplayTasksComponent(), LogGameplayTasks, Error, TEXT( "%s> re-Activating Finished task!" ), *GetName() );
                }
            }
            break;

        default:
            checkNoEntry();
            break;
    }
}

void USVOAITask_MoveTo::ConditionalPerformMove()
{
    if ( MoveRequest.IsUsingPathfinding() && OwnerController && OwnerController->ShouldPostponePathUpdates() )
    {
        UE_VLOG( GetGameplayTasksComponent(), LogGameplayTasks, Log, TEXT( "%s> can't path right now, waiting..." ), *GetName() );
        OwnerController->GetWorldTimerManager().SetTimer( MoveRetryTimerHandle, this, &USVOAITask_MoveTo::ConditionalPerformMove, 0.2f, false );
    }
    else
    {
        MoveRetryTimerHandle.Invalidate();
        PerformMove();
    }
}

void USVOAITask_MoveTo::SetUp( AAIController * controller, const FAIMoveRequest & move_request )
{
    OwnerController = controller;
    MoveRequest = move_request;

    // Fail if no nav component
    //    myNavComponent = Cast< USVONNavigationComponent >( GetOwnerActor()->GetComponentByClass( USVONNavigationComponent::StaticClass() ) );
    //    if ( !myNavComponent )
    //    {
    //#if WITH_EDITOR
    //        UE_VLOG( this, VUESVON, Error, TEXT( "SVONMoveTo request failed due missing SVONNavComponent" ), *MoveRequest.ToString() );
    //        UE_LOG( UESVON, Error, TEXT( "SVONMoveTo request failed due missing SVONNavComponent on the pawn" ) );
    //        return;
    //#endif
    //    }
    //    // Use the path instance from the navcomponent
    //    mySVONPath = myNavComponent->GetPath();
}

void USVOAITask_MoveTo::SetContinuousGoalTracking( const bool continuous_goal_tracking )
{
    UseContinuousTracking = continuous_goal_tracking;
}

void USVOAITask_MoveTo::SetAsyncPathFinding( const bool async_pathfinding )
{
    UseAsyncPathFinding = async_pathfinding;
}

USVOAITask_MoveTo * USVOAITask_MoveTo::AIMoveToInSVO( AAIController * controller, FVector goal_location, AActor * goal_actor, const float acceptance_radius, const bool async_pathfinding, const EAIOptionFlag::Type stop_on_overlap, const bool lock_ai_logic, const bool continuous_goal_tracking )
{
    USVOAITask_MoveTo * move_to_task = controller ? UAITask::NewAITask< USVOAITask_MoveTo >( *controller, EAITaskPriority::High ) : nullptr;
    if ( move_to_task != nullptr )
    {
        FAIMoveRequest move_request;
        if ( goal_actor != nullptr )
        {
            move_request.SetGoalActor( goal_actor );
        }
        else
        {
            move_request.SetGoalLocation( goal_location );
        }

        move_request.SetAcceptanceRadius( acceptance_radius );
        move_request.SetReachTestIncludesAgentRadius( FAISystem::PickAIOption( stop_on_overlap, move_request.IsReachTestIncludingAgentRadius() ) );
        move_request.SetAllowPartialPath( false );
        move_request.SetUsePathfinding( false );
        move_request.SetProjectGoalLocation( false );

        move_to_task->SetAsyncPathFinding( async_pathfinding );

        if ( controller != nullptr )
        {
            move_request.SetNavigationFilter( controller->GetDefaultNavigationFilterClass() );
        }

        move_to_task->SetUp( controller, move_request );
        move_to_task->SetContinuousGoalTracking( continuous_goal_tracking );

        if ( lock_ai_logic )
        {
            move_to_task->RequestAILogicLocking();
        }
    }

    return move_to_task;
}

UPathFollowingComponent * USVOAITask_MoveTo::GetPathFollowingComponent() const
{
    return OwnerController ? OwnerController->GetPathFollowingComponent() : nullptr;
}

void USVOAITask_MoveTo::FinishMoveTask( EPathFollowingResult::Type result )
{
    if ( MoveRequestID.IsValid() )
    {
        UPathFollowingComponent * path_following_component = GetPathFollowingComponent();
        if ( path_following_component && path_following_component->GetStatus() != EPathFollowingStatus::Idle )
        {
            ResetObservers();
            path_following_component->AbortMove( *this, FPathFollowingResultFlags::OwnerFinished, MoveRequestID );
        }
    }

    MoveResult = result;
    EndTask();

    if ( result == EPathFollowingResult::Invalid )
    {
        OnRequestFailed.Broadcast();
    }
    else
    {
        OnMoveFinishedDelegate.Broadcast( result, OwnerController );
    }
}

void USVOAITask_MoveTo::ResetObservers()
{
    if ( Path.IsValid() )
    {
        Path->DisableGoalActorObservation();
    }

    if ( PathFinishDelegateHandle.IsValid() )
    {
        if ( UPathFollowingComponent * path_following_component = GetPathFollowingComponent() )
        {
            path_following_component->OnRequestFinished.Remove( PathFinishDelegateHandle );
        }

        PathFinishDelegateHandle.Reset();
    }

    if ( PathUpdateDelegateHandle.IsValid() )
    {
        if ( Path.IsValid() )
        {
            Path->RemoveObserver( PathUpdateDelegateHandle );
        }

        PathUpdateDelegateHandle.Reset();
    }
}

void USVOAITask_MoveTo::ResetTimers()
{
    if ( OwnerController )
    {
        // Remove all timers including the ones that might have been set with SetTimerForNextTick
        OwnerController->GetWorldTimerManager().ClearAllTimersForObject( this );
    }
    MoveRetryTimerHandle.Invalidate();
    PathRetryTimerHandle.Invalidate();
}

void USVOAITask_MoveTo::Activate()
{
    Super::Activate();

    UE_CVLOG( UseContinuousTracking, GetGameplayTasksComponent(), LogGameplayTasks, Log, TEXT( "Continuous goal tracking requested, moving to: %s" ), MoveRequest.IsMoveToActorRequest() ? TEXT( "actor => looping successful moves!" ) : TEXT( "location => will NOT loop" ) );

    MoveRequestID = FAIRequestID::InvalidRequest;
    ConditionalPerformMove();
}

void USVOAITask_MoveTo::OnDestroy( bool owner_finished )
{
    Super::OnDestroy( owner_finished );

    ResetObservers();
    ResetTimers();

    if ( MoveRequestID.IsValid() )
    {
        UPathFollowingComponent * path_following_component = GetPathFollowingComponent();
        if ( path_following_component && path_following_component->GetStatus() != EPathFollowingStatus::Idle )
        {
            path_following_component->AbortMove( *this, FPathFollowingResultFlags::OwnerFinished, MoveRequestID );
        }
    }

    // clear the shared pointer now to make sure other systems
    // don't think this path is still being used
    Path = nullptr;
}

void USVOAITask_MoveTo::Pause()
{
    if ( OwnerController != nullptr && MoveRequestID.IsValid() )
    {
        OwnerController->PauseMove( MoveRequestID );
    }

    ResetTimers();
    Super::Pause();
}

void USVOAITask_MoveTo::Resume()
{
    Super::Resume();

    if ( !MoveRequestID.IsValid() || ( OwnerController != nullptr && !OwnerController->ResumeMove( MoveRequestID ) ) )
    {
        UE_CVLOG( MoveRequestID.IsValid(), GetGameplayTasksComponent(), LogGameplayTasks, Log, TEXT( "%s> Resume move failed, starting new one." ), *GetName() );
        ConditionalPerformMove();
    }
}

void USVOAITask_MoveTo::SetObservedPath( FNavPathSharedPtr path )
{
    if ( PathUpdateDelegateHandle.IsValid() && Path.IsValid() )
    {
        Path->RemoveObserver( PathUpdateDelegateHandle );
    }

    PathUpdateDelegateHandle.Reset();

    Path = path;
    if ( Path.IsValid() )
    {
        // disable auto repaths, it will be handled by move task to include ShouldPostponePathUpdates condition
        Path->EnableRecalculationOnInvalidation( false );
        PathUpdateDelegateHandle = Path->AddObserver( FNavigationPath::FPathObserverDelegate::FDelegate::CreateUObject( this, &USVOAITask_MoveTo::OnPathEvent ) );
    }
}

void USVOAITask_MoveTo::OnPathEvent( FNavigationPath * path, ENavPathEvent::Type path_event )
{
    const static UEnum * NavPathEventEnum = StaticEnum< ENavPathEvent::Type >();
    UE_VLOG( GetGameplayTasksComponent(), LogGameplayTasks, Log, TEXT( "%s> Path event: %s" ), *GetName(), *NavPathEventEnum->GetNameStringByValue( path_event ) );

    switch ( path_event )
    {
        case ENavPathEvent::NewPath:
        case ENavPathEvent::UpdatedDueToGoalMoved:
        case ENavPathEvent::UpdatedDueToNavigationChanged:
            if ( path != nullptr && path->IsPartial() && !MoveRequest.IsUsingPartialPaths() )
            {
                UE_VLOG( GetGameplayTasksComponent(), LogGameplayTasks, Log, TEXT( ">> partial path is not allowed, aborting" ) );
                UPathFollowingComponent::LogPathHelper( OwnerController, path, MoveRequest.GetGoalActor() );
                FinishMoveTask( EPathFollowingResult::Aborted );
            }
#if ENABLE_VISUAL_LOG
            else if ( !IsActive() )
            {
                UPathFollowingComponent::LogPathHelper( OwnerController, path, MoveRequest.GetGoalActor() );
            }
#endif // ENABLE_VISUAL_LOG
            break;

        case ENavPathEvent::Invalidated:
            ConditionalUpdatePath();
            break;

        case ENavPathEvent::Cleared:
        case ENavPathEvent::RePathFailed:
            UE_VLOG( GetGameplayTasksComponent(), LogGameplayTasks, Log, TEXT( ">> no path, aborting!" ) );
            FinishMoveTask( EPathFollowingResult::Aborted );
            break;

        case ENavPathEvent::MetaPathUpdate:
        default:
            break;
    }
}

void USVOAITask_MoveTo::ConditionalUpdatePath()
{
    // mark this path as waiting for repath so that PathFollowingComponent doesn't abort the move while we
    // micro manage repathing moment
    // note that this flag fill get cleared upon repathing end
    if ( Path.IsValid() )
    {
        Path->SetManualRepathWaiting( true );
    }

    if ( MoveRequest.IsUsingPathfinding() && OwnerController && OwnerController->ShouldPostponePathUpdates() )
    {
        UE_VLOG( GetGameplayTasksComponent(), LogGameplayTasks, Log, TEXT( "%s> can't path right now, waiting..." ), *GetName() );
        OwnerController->GetWorldTimerManager().SetTimer( PathRetryTimerHandle, this, &USVOAITask_MoveTo::ConditionalUpdatePath, 0.2f, false );
    }
    else
    {
        PathRetryTimerHandle.Invalidate();

        ANavigationData * NavData = Path.IsValid() ? Path->GetNavigationDataUsed() : nullptr;
        if ( NavData )
        {
            NavData->RequestRePath( Path, ENavPathUpdateType::NavigationChanged );
        }
        else
        {
            UE_VLOG( GetGameplayTasksComponent(), LogGameplayTasks, Log, TEXT( "%s> unable to repath, aborting!" ), *GetName() );
            FinishMoveTask( EPathFollowingResult::Aborted );
        }
    }
}

void USVOAITask_MoveTo::OnRequestFinished( FAIRequestID request_id, const FPathFollowingResult & result )
{
    if ( request_id == MoveRequestID )
    {
        if ( result.HasFlag( FPathFollowingResultFlags::UserAbort ) && result.HasFlag( FPathFollowingResultFlags::NewRequest ) && !result.HasFlag( FPathFollowingResultFlags::ForcedScript ) )
        {
            UE_VLOG( GetGameplayTasksComponent(), LogGameplayTasks, Log, TEXT( "%s> ignoring OnRequestFinished, move was aborted by new request" ), *GetName() );
        }
        else
        {
            // reset request Id, FinishMoveTask doesn't need to update path following's state
            MoveRequestID = FAIRequestID::InvalidRequest;

            if ( UseContinuousTracking && MoveRequest.IsMoveToActorRequest() && result.IsSuccess() )
            {
                UE_VLOG( GetGameplayTasksComponent(), LogGameplayTasks, Log, TEXT( "%s> received OnRequestFinished and goal tracking is active! Moving again in next tick" ), *GetName() );
                GetWorld()->GetTimerManager().SetTimerForNextTick( this, &USVOAITask_MoveTo::PerformMove );
            }
            else
            {
                FinishMoveTask( result.Code );
            }
        }
    }
    else if ( IsActive() )
    {
        UE_VLOG( GetGameplayTasksComponent(), LogGameplayTasks, Warning, TEXT( "%s> received OnRequestFinished with not matching RequestID!" ), *GetName() );
    }
}

bool USVOAITask_MoveTo::BuildPathfindingQuery( FSVOPathFindingQuery & path_finding_query, const FAIMoveRequest & move_request ) const
{
    bool bResult = false;

    if ( auto * svo_navigation_system = GEngine->GetEngineSubsystem< USVONavigationSystem >() )
    {
        const auto & navigation_data = svo_navigation_system->GetNavigationData();

        if ( navigation_data.IsValid() )
        {
            const auto goal_location = MoveRequest.IsMoveToActorRequest()
                                           ? MoveRequest.GetGoalActor()->GetActorLocation()
                                           : MoveRequest.GetGoalLocation();

            path_finding_query = FSVOPathFindingQuery( this, *navigation_data.Get(), OwnerController->GetPawn()->GetActorLocation(), goal_location );
            bResult = true;
        }
    }
    else
    {
        UE_VLOG( GetGameplayTasksComponent(), LogGameplayTasks, Warning, TEXT( "Unable to find NavigationData instance while calling AAIController::BuildPathfindingQuery" ) );
    }

    return bResult;
}

void USVOAITask_MoveTo::FindPathForMoveRequest( const FAIMoveRequest & move_request, const FSVOPathFindingQuery & path_finding_query, FNavPathSharedPtr & path ) const
{
    if ( auto * svo_navigation_system = GEngine->GetEngineSubsystem< USVONavigationSystem >() )
    {
        const auto path_finding_result = svo_navigation_system->FindPathSync( path_finding_query );
        if ( path_finding_result.Result != ENavigationQueryResult::Error )
        {
            if ( path_finding_result.IsSuccessful() && path_finding_result.Path.IsValid() )
            {
                if ( move_request.IsMoveToActorRequest() )
                {
                    path_finding_result.Path->SetGoalActorObservation( *move_request.GetGoalActor(), 100.0f );
                }

                path_finding_result.Path->EnableRecalculationOnInvalidation( true );
                path = path_finding_result.Path;
            }
        }
        else
        {
            UE_VLOG( GetGameplayTasksComponent(), LogGameplayTasks, Error, TEXT( "Trying to find path to %s resulted in Error" ), move_request.IsMoveToActorRequest() ? *GetNameSafe( move_request.GetGoalActor() ) : *move_request.GetGoalLocation().ToString() );
            UE_VLOG_SEGMENT( GetGameplayTasksComponent(), LogGameplayTasks, Error, OwnerController->GetPawn() ? OwnerController->GetPawn()->GetActorLocation() : FAISystem::InvalidLocation, move_request.GetGoalLocation(), FColor::Red, TEXT( "Failed move to %s" ), *GetNameSafe( move_request.GetGoalActor() ) );
        }
    }
}

FPathFollowingRequestResult USVOAITask_MoveTo::MoveTo( FNavPathSharedPtr * out_path, const FAIMoveRequest & move_request )
{
    UE_VLOG( GetGameplayTasksComponent(), LogGameplayTasks, Log, TEXT( "SVO MoveTo: %s" ), *move_request.ToString() );

    FPathFollowingRequestResult request_result;
    request_result.Code = EPathFollowingRequestResult::Failed;

    if ( move_request.IsValid() == false )
    {
        UE_VLOG( GetGameplayTasksComponent(), LogGameplayTasks, Error, TEXT( "USVOAITask_MoveTo::MoveTo: request failed due move_request not being valid. Most probably desireg Goal Actor not longer exists" ), *move_request.ToString() );
        return request_result;
    }

    auto * path_following_component = GetPathFollowingComponent();
    if ( path_following_component == nullptr )
    {
        UE_VLOG( GetGameplayTasksComponent(), LogGameplayTasks, Error, TEXT( "USVOAITask_MoveTo::MoveTo: request failed due missing PathFollowingComponent" ) );
        return request_result;
    }

    bool can_request_move = true;
    bool is_already_at_goal;

    if ( !move_request.IsMoveToActorRequest() )
    {
        if ( move_request.GetGoalLocation().ContainsNaN() || FAISystem::IsValidLocation( move_request.GetGoalLocation() ) == false )
        {
            UE_VLOG( GetGameplayTasksComponent(), LogGameplayTasks, Error, TEXT( "USVOAITask_MoveTo::MoveTo: Destination is not valid! Goal(%s)" ), TEXT_AI_LOCATION( MoveRequest.GetGoalLocation() ) );
            can_request_move = false;
        }

        is_already_at_goal = can_request_move && path_following_component->HasReached( move_request );
    }
    else
    {
        is_already_at_goal = can_request_move && path_following_component->HasReached( move_request );
    }

    if ( is_already_at_goal )
    {
        UE_VLOG( GetGameplayTasksComponent(), LogGameplayTasks, Log, TEXT( "USVOAITask_MoveTo::MoveTo: already at goal!" ) );
        request_result.MoveId = path_following_component->RequestMoveWithImmediateFinish( EPathFollowingResult::Success );
        request_result.Code = EPathFollowingRequestResult::AlreadyAtGoal;
    }
    else if ( can_request_move )
    {
        if ( UseAsyncPathFinding )
        {
            // run the async task which will call RequestMove when done
            request_result.MoveId = FAIRequestID::InvalidRequest;
            request_result.Code = EPathFollowingRequestResult::RequestSuccessful;
        }
        else
        {
            FSVOPathFindingQuery path_finding_query;
            const bool is_valid_query = BuildPathfindingQuery( path_finding_query, move_request );

            if ( is_valid_query )
            {
                FNavPathSharedPtr new_path;
                FindPathForMoveRequest( move_request, path_finding_query, new_path );

                const FAIRequestID request_id = new_path.IsValid()
                                                    ? RequestMove( move_request, new_path )
                                                    : FAIRequestID::InvalidRequest;

                if ( request_id.IsValid() )
                {
                    request_result.MoveId = request_id;
                    request_result.Code = EPathFollowingRequestResult::RequestSuccessful;

                    if ( out_path != nullptr )
                    {
                        *out_path = new_path;
                    }
                }
            }
        }
    }

    if ( request_result.Code == EPathFollowingRequestResult::Failed )
    {
        request_result.MoveId = path_following_component->RequestMoveWithImmediateFinish( EPathFollowingResult::Invalid );
    }

    return request_result;
}

FAIRequestID USVOAITask_MoveTo::RequestMove( const FAIMoveRequest & move_request, FNavPathSharedPtr path ) const
{
    uint32 request_id = FAIRequestID::InvalidRequest;
    if ( auto * path_following_component = GetPathFollowingComponent() )
    {
        request_id = path_following_component->RequestMove( move_request, path );
    }

    return request_id;
}

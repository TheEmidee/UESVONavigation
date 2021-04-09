#include "SVOBTTask_MoveTo.h"

#include "SVOAITask_MoveTo.h"

#include <AIController.h>
#include <BehaviorTree/Blackboard/BlackboardKeyType_Object.h>
#include <BehaviorTree/Blackboard/BlackboardKeyType_Vector.h>
#include <BehaviorTree/BlackboardComponent.h>
#include <Perception/AIPerceptionComponent.h>
#include <Runtime/Engine/Public/VisualLogger/VisualLogger.h>

USVOBTTask_MoveTo::USVOBTTask_MoveTo( const FObjectInitializer & object_initializer ) :
    Super( object_initializer )
{
    NodeName = "SVO Move To";
    bUseGameplayTasks = GET_AI_CONFIG_VAR( bEnableBTAITasks );
    bNotifyTick = !bUseGameplayTasks;
    bNotifyTaskFinished = true;

    AcceptableRadius = GET_AI_CONFIG_VAR( AcceptanceRadius );
    bReachTestIncludesGoalRadius = bReachTestIncludesAgentRadius = GET_AI_CONFIG_VAR( bFinishMoveOnGoalOverlap );
    bTrackMovingGoal = true;
    ObservedBlackboardValueTolerance = AcceptableRadius * 0.95f;
    bUseAsyncPathfinding = false;

    // accept only actors and vectors
    BlackboardKey.AddObjectFilter( this, GET_MEMBER_NAME_CHECKED( USVOBTTask_MoveTo, BlackboardKey ), AActor::StaticClass() );
    BlackboardKey.AddVectorFilter( this, GET_MEMBER_NAME_CHECKED( USVOBTTask_MoveTo, BlackboardKey ) );
}

EBTNodeResult::Type USVOBTTask_MoveTo::ExecuteTask( UBehaviorTreeComponent & owner_component, uint8 * node_memory )
{
    EBTNodeResult::Type node_result = EBTNodeResult::InProgress;

    FSVOBTTaskMoveToMemory * memory = reinterpret_cast< FSVOBTTaskMoveToMemory * >( node_memory );
    memory->PreviousGoalLocation = FAISystem::InvalidLocation;
    memory->MoveRequestID = FAIRequestID::InvalidRequest;

    AAIController * controller = owner_component.GetAIOwner();
    memory->bWaitingForPath = bUseGameplayTasks ? false : controller->ShouldPostponePathUpdates();
    if ( !memory->bWaitingForPath )
    {
        node_result = PerformMoveTask( owner_component, node_memory );
    }
    else
    {
        UE_VLOG( controller, LogBehaviorTree, Log, TEXT( "Pathfinding requests are freezed, waiting..." ) );
    }

    if ( node_result == EBTNodeResult::InProgress && bObserveBlackboardValue )
    {
        UBlackboardComponent * blackboard_component = owner_component.GetBlackboardComponent();
        if ( ensure( blackboard_component != nullptr ) )
        {
            if ( memory->BBObserverDelegateHandle.IsValid() )
            {
                UE_VLOG( controller, LogBehaviorTree, Warning, TEXT( "USVOBTTask_MoveTo::ExecuteTask \'%s\' Old BBObserverDelegateHandle is still valid! Removing old Observer." ), *GetNodeName() );
                blackboard_component->UnregisterObserver( BlackboardKey.GetSelectedKeyID(), memory->BBObserverDelegateHandle );
            }
            memory->BBObserverDelegateHandle = blackboard_component->RegisterObserver( BlackboardKey.GetSelectedKeyID(), this, FOnBlackboardChangeNotification::CreateUObject( this, &USVOBTTask_MoveTo::OnBlackboardValueChange ) );
        }
    }

    return node_result;
}

EBTNodeResult::Type USVOBTTask_MoveTo::AbortTask( UBehaviorTreeComponent & owner_component, uint8 * node_memory )
{
    FSVOBTTaskMoveToMemory * my_memory = reinterpret_cast< FSVOBTTaskMoveToMemory * >( node_memory );
    if ( !my_memory->bWaitingForPath )
    {
        if ( my_memory->MoveRequestID.IsValid() )
        {
            if ( AAIController * ai_controller = owner_component.GetAIOwner() )
            {
                if ( ai_controller && ai_controller->GetPathFollowingComponent() )
                {
                    ai_controller->GetPathFollowingComponent()->AbortMove( *this, FPathFollowingResultFlags::OwnerFinished, my_memory->MoveRequestID );
                }
            }
        }
        else
        {
            my_memory->bObserverCanFinishTask = false;
            USVOAITask_MoveTo * move_task = my_memory->Task.Get();
            if ( move_task )
            {
                move_task->ExternalCancel();
            }
            else
            {
                UE_VLOG( &owner_component, LogBehaviorTree, Error, TEXT( "Can't abort path following! bWaitingForPath:false, MoveRequestID:invalid, MoveTask:none!" ) );
            }
        }
    }

    return Super::AbortTask( owner_component, node_memory );
}

void USVOBTTask_MoveTo::OnTaskFinished( UBehaviorTreeComponent & owner_component, uint8 * node_memory, EBTNodeResult::Type task_result )
{
    FSVOBTTaskMoveToMemory * memory = reinterpret_cast< FSVOBTTaskMoveToMemory * >( node_memory );
    memory->Task.Reset();

    if ( bObserveBlackboardValue )
    {
        UBlackboardComponent * blackboard_component = owner_component.GetBlackboardComponent();
        if ( ensure( blackboard_component ) && memory->BBObserverDelegateHandle.IsValid() )
        {
            blackboard_component->UnregisterObserver( BlackboardKey.GetSelectedKeyID(), memory->BBObserverDelegateHandle );
        }

        memory->BBObserverDelegateHandle.Reset();
    }

    Super::OnTaskFinished( owner_component, node_memory, task_result );
}

void USVOBTTask_MoveTo::TickTask( UBehaviorTreeComponent & owner_component, uint8 * node_memory, float delta_seconds )
{
    FSVOBTTaskMoveToMemory * memory = reinterpret_cast< FSVOBTTaskMoveToMemory * >( node_memory );
    if ( memory->bWaitingForPath && !owner_component.IsPaused() )
    {
        AAIController * controller = owner_component.GetAIOwner();
        if ( controller && !controller->ShouldPostponePathUpdates() )
        {
            UE_VLOG( controller, LogBehaviorTree, Log, TEXT( "Pathfinding requests are unlocked!" ) );
            memory->bWaitingForPath = false;

            const EBTNodeResult::Type node_result = PerformMoveTask( owner_component, node_memory );
            if ( node_result != EBTNodeResult::InProgress )
            {
                FinishLatentTask( owner_component, node_result );
            }
        }
    }
}

uint16 USVOBTTask_MoveTo::GetInstanceMemorySize() const
{
    return sizeof( FSVOBTTaskMoveToMemory );
}

void USVOBTTask_MoveTo::OnGameplayTaskDeactivated( UGameplayTask & task )
{
    // AI move task finished
    USVOAITask_MoveTo * move_task = Cast< USVOAITask_MoveTo >( &task );
    if ( move_task && move_task->GetAIController() && move_task->GetState() != EGameplayTaskState::Paused )
    {
        UBehaviorTreeComponent * behavior_component = GetBTComponentForTask( task );
        if ( behavior_component )
        {
            uint8 * RawMemory = behavior_component->GetNodeMemory( this, behavior_component->FindInstanceContainingNode( this ) );
            FSVOBTTaskMoveToMemory * memory = reinterpret_cast< FSVOBTTaskMoveToMemory * >( RawMemory );

            if ( memory->bObserverCanFinishTask && ( move_task == memory->Task ) )
            {
                const bool bSuccess = move_task->WasMoveSuccessful();
                FinishLatentTask( *behavior_component, bSuccess ? EBTNodeResult::Succeeded : EBTNodeResult::Failed );
            }
        }
    }
}

void USVOBTTask_MoveTo::OnMessage( UBehaviorTreeComponent & owner_component, uint8 * node_memory, FName message, int32 request_id, bool success )
{
    // AIMessage_RepathFailed means task has failed
    success &= ( message != UBrainComponent::AIMessage_RepathFailed );
    Super::OnMessage( owner_component, node_memory, message, request_id, success );
}

FString USVOBTTask_MoveTo::GetStaticDescription() const
{
    FString key_description( "invalid" );

    if ( BlackboardKey.SelectedKeyType == UBlackboardKeyType_Object::StaticClass() ||
         BlackboardKey.SelectedKeyType == UBlackboardKeyType_Vector::StaticClass() )
    {
        key_description = BlackboardKey.SelectedKeyName.ToString();
    }

    return FString::Printf( TEXT( "%s: %s" ), *Super::GetStaticDescription(), *key_description );
}

#if WITH_EDITOR
FName USVOBTTask_MoveTo::GetNodeIconName() const
{
    return FName( "BTEditor.Graph.BTNode.Task.MoveTo.Icon" );
}
#endif

EBlackboardNotificationResult USVOBTTask_MoveTo::OnBlackboardValueChange( const UBlackboardComponent & blackboard, FBlackboard::FKey change_key_id )
{
    UBehaviorTreeComponent * behaviour_tree_component = Cast< UBehaviorTreeComponent >( blackboard.GetBrainComponent() );
    if ( behaviour_tree_component == nullptr )
    {
        return EBlackboardNotificationResult::RemoveObserver;
    }

    uint8 * raw_memory = behaviour_tree_component->GetNodeMemory( this, behaviour_tree_component->FindInstanceContainingNode( this ) );
    FSVOBTTaskMoveToMemory * memory = reinterpret_cast< FSVOBTTaskMoveToMemory * >( raw_memory );

    const EBTTaskStatus::Type task_status = behaviour_tree_component->GetTaskStatus( this );
    if ( task_status != EBTTaskStatus::Active )
    {
        UE_VLOG( behaviour_tree_component, LogBehaviorTree, Error, TEXT( "BT MoveTo \'%s\' task observing BB entry while no longer being active!" ), *GetNodeName() );

        // resetting BBObserverDelegateHandle without unregistering observer since
        // returning EBlackboardNotificationResult::RemoveObserver here will take care of that for us
        memory->BBObserverDelegateHandle.Reset(); //-V595

        return EBlackboardNotificationResult::RemoveObserver;
    }

    // this means the move has already started. MyMemory->bWaitingForPath == true would mean we're waiting for right moment to start it anyway,
    // so we don't need to do anything due to BB value change
    if ( memory != nullptr && memory->bWaitingForPath == false && behaviour_tree_component->GetAIOwner() != nullptr )
    {
        check( behaviour_tree_component->GetAIOwner()->GetPathFollowingComponent() );

        bool must_update_move = true;
        // check if new goal is almost identical to previous one
        if ( BlackboardKey.SelectedKeyType == UBlackboardKeyType_Vector::StaticClass() )
        {
            const FVector TargetLocation = blackboard.GetValue< UBlackboardKeyType_Vector >( BlackboardKey.GetSelectedKeyID() );

            must_update_move = ( FVector::DistSquared( TargetLocation, memory->PreviousGoalLocation ) > FMath::Square( ObservedBlackboardValueTolerance ) );
        }

        if ( must_update_move )
        {
            // don't abort move if using AI tasks - it will mess things up
            if ( memory->MoveRequestID.IsValid() )
            {
                UE_VLOG( behaviour_tree_component, LogBehaviorTree, Log, TEXT( "blackboard value for goal has changed, aborting current move request" ) );
                StopWaitingForMessages( *behaviour_tree_component );
                behaviour_tree_component->GetAIOwner()->GetPathFollowingComponent()->AbortMove( *this, FPathFollowingResultFlags::NewRequest, memory->MoveRequestID, EPathFollowingVelocityMode::Keep );
            }

            if ( !bUseGameplayTasks && behaviour_tree_component->GetAIOwner()->ShouldPostponePathUpdates() )
            {
                // NodeTick will take care of requesting move
                memory->bWaitingForPath = true;
            }
            else
            {
                const EBTNodeResult::Type node_result = PerformMoveTask( *behaviour_tree_component, raw_memory );
                if ( node_result != EBTNodeResult::InProgress )
                {
                    FinishLatentTask( *behaviour_tree_component, node_result );
                }
            }
        }
    }

    return EBlackboardNotificationResult::ContinueObserving;
}

EBTNodeResult::Type USVOBTTask_MoveTo::PerformMoveTask( UBehaviorTreeComponent & owner_component, uint8 * node_memory )
{
    const UBlackboardComponent * blackboard_component = owner_component.GetBlackboardComponent();
    FSVOBTTaskMoveToMemory * memory = reinterpret_cast< FSVOBTTaskMoveToMemory * >( node_memory );
    AAIController * controller = owner_component.GetAIOwner();

    EBTNodeResult::Type node_result = EBTNodeResult::Failed;
    if ( controller && blackboard_component )
    {
        FAIMoveRequest move_request;
        move_request.SetNavigationFilter( controller->GetDefaultNavigationFilterClass() );
        move_request.SetAcceptanceRadius( AcceptableRadius );
        move_request.SetReachTestIncludesAgentRadius( bReachTestIncludesAgentRadius );
        move_request.SetReachTestIncludesGoalRadius( bReachTestIncludesGoalRadius );
        move_request.SetUsePathfinding( true );

        if ( BlackboardKey.SelectedKeyType == UBlackboardKeyType_Object::StaticClass() )
        {
            UObject * key_value = blackboard_component->GetValue< UBlackboardKeyType_Object >( BlackboardKey.GetSelectedKeyID() );
            AActor * target_actor = Cast< AActor >( key_value );

            if ( target_actor )
            {
                if ( bTrackMovingGoal )
                {
                    move_request.SetGoalActor( target_actor );
                }
                else
                {
                    move_request.SetGoalLocation( target_actor->GetActorLocation() );
                }
            }
            else
            {
                UE_VLOG( controller, LogBehaviorTree, Warning, TEXT( "UBTTask_MoveTo::ExecuteTask tried to go to actor while BB %s entry was empty" ), *BlackboardKey.SelectedKeyName.ToString() );
            }
        }
        else if ( BlackboardKey.SelectedKeyType == UBlackboardKeyType_Vector::StaticClass() )
        {
            const FVector target_location = blackboard_component->GetValue< UBlackboardKeyType_Vector >( BlackboardKey.GetSelectedKeyID() );
            move_request.SetGoalLocation( target_location );

            memory->PreviousGoalLocation = target_location;
        }

        if ( move_request.IsValid() )
        {
            if ( GET_AI_CONFIG_VAR( bEnableBTAITasks ) )
            {
                USVOAITask_MoveTo * move_task = memory->Task.Get();
                const bool bReuseExistingTask = ( move_task != nullptr );

                move_task = PrepareMoveTask( owner_component, move_task, move_request );

                if ( move_task != nullptr )
                {
                    memory->bObserverCanFinishTask = false;

                    if ( bReuseExistingTask )
                    {
                        if ( move_task->IsActive() )
                        {
                            UE_VLOG( controller, LogBehaviorTree, Verbose, TEXT( "\'%s\' reusing AITask %s" ), *GetNodeName(), *move_task->GetName() );
                            move_task->ConditionalPerformMove();
                        }
                        else
                        {
                            UE_VLOG( controller, LogBehaviorTree, Verbose, TEXT( "\'%s\' reusing AITask %s, but task is not active - handing over move performing to task mechanics" ), *GetNodeName(), *move_task->GetName() );
                        }
                    }
                    else
                    {
                        memory->Task = move_task;
                        UE_VLOG( controller, LogBehaviorTree, Verbose, TEXT( "\'%s\' task implementing move with task %s" ), *GetNodeName(), *move_task->GetName() );
                        move_task->ReadyForActivation();
                    }

                    memory->bObserverCanFinishTask = true;
                    node_result = ( move_task->GetState() != EGameplayTaskState::Finished ) ? EBTNodeResult::InProgress : move_task->WasMoveSuccessful() ? EBTNodeResult::Succeeded : EBTNodeResult::Failed;
                }
            }
            else
            {
                const FPathFollowingRequestResult request_result = controller->MoveTo( move_request );
                if ( request_result.Code == EPathFollowingRequestResult::RequestSuccessful )
                {
                    memory->MoveRequestID = request_result.MoveId;
                    WaitForMessage( owner_component, UBrainComponent::AIMessage_MoveFinished, request_result.MoveId );
                    WaitForMessage( owner_component, UBrainComponent::AIMessage_RepathFailed );

                    node_result = EBTNodeResult::InProgress;
                }
                else if ( request_result.Code == EPathFollowingRequestResult::AlreadyAtGoal )
                {
                    node_result = EBTNodeResult::Succeeded;
                }
            }
        }
    }

    return node_result;
}

USVOAITask_MoveTo * USVOBTTask_MoveTo::PrepareMoveTask( UBehaviorTreeComponent & owner_component, USVOAITask_MoveTo * existing_task, FAIMoveRequest & move_request )
{
    USVOAITask_MoveTo * move_task = existing_task ? existing_task : NewBTAITask< USVOAITask_MoveTo >( owner_component );
    if ( move_task )
    {
        move_task->SetUp( move_task->GetAIController(), move_request );
    }

    return move_task;
}
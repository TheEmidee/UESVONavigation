#include "SVOAITask_MoveTo.h"


#include "AIResources.h"
#include "Perception/AIPerceptionComponent.h"


#include <AIController.h>
#include <Runtime/Engine/Public/VisualLogger/VisualLogger.h>
#include <Runtime/GameplayTasks/Classes/GameplayTasksComponent.h>

USVOAITask_MoveTo::USVOAITask_MoveTo( const FObjectInitializer & object_initializer ) :
    Super( object_initializer )
{
    bIsPausable = true;
    //MoveRequestID = FAIRequestID::InvalidRequest;

    MoveRequest.SetAcceptanceRadius( GET_AI_CONFIG_VAR( AcceptanceRadius ) );
    MoveRequest.SetReachTestIncludesAgentRadius( GET_AI_CONFIG_VAR( bFinishMoveOnGoalOverlap ) );
    MoveRequest.SetAllowPartialPath( GET_AI_CONFIG_VAR( bAcceptPartialPaths ) );
    MoveRequest.SetUsePathfinding( true );

    //myResult.Code = ESVONPathfindingRequestResult::Failed;

    AddRequiredResource( UAIResource_Movement::StaticClass() );
    AddClaimedResource( UAIResource_Movement::StaticClass() );

    MoveResult = EPathFollowingResult::Invalid;
    /*bUseContinuousTracking = false;

    Path = MakeShareable< FNavigationPath >( new FNavigationPath() );*/
}

bool USVOAITask_MoveTo::WasMoveSuccessful() const
{
    return MoveResult == EPathFollowingResult::Success;
}

void USVOAITask_MoveTo::PerformMove()
{
    // Prepare the move first (check for early out)
    //CheckPathPreConditions();

    //ResetObservers();
    //ResetTimers();
    //ResetPaths();

    //if ( myResult.Code == ESVONPathfindingRequestResult::AlreadyAtGoal )
    //{
    //    MoveRequestID = myResult.MoveId;
    //    OnRequestFinished( myResult.MoveId, FPathFollowingResult( EPathFollowingResult::Success, FPathFollowingResultFlags::AlreadyAtGoal ) );
    //    return;
    //}

    //// If we're ready to path, then request the path
    //if ( myResult.Code == ESVONPathfindingRequestResult::ReadyToPath )
    //{

    //    myUseAsyncPathfinding ? RequestPathAsync() : RequestPathSynchronous();

    //    switch ( myResult.Code )
    //    {
    //        case ESVONPathfindingRequestResult::Failed:
    //            FinishMoveTask( EPathFollowingResult::Invalid );
    //            break;
    //        case ESVONPathfindingRequestResult::Success: // Synchronous pathfinding
    //            MoveRequestID = myResult.MoveId;
    //            if ( IsFinished() )
    //                UE_VLOG( GetGameplayTasksComponent(), LogGameplayTasks, Error, TEXT( "%s> re-Activating Finished task!" ), *GetName() );
    //            RequestMove(); // Start the move
    //            break;
    //        case ESVONPathfindingRequestResult::Deferred: // Async...we're waiting on the task to return
    //            MoveRequestID = myResult.MoveId;
    //            myAsyncTaskComplete = false;
    //            break;
    //        default:
    //            checkNoEntry();
    //            break;
    //    }
    //}
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

void USVOAITask_MoveTo::SetUp( AAIController * controller, const FAIMoveRequest & move_request, bool use_async_pathfinding )
{
    OwnerController = controller;
    MoveRequest = move_request;
    UseAsyncPathfinding = use_async_pathfinding;
    bTickingTask = UseAsyncPathfinding;

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

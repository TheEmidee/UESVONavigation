#pragma once

#include "SVONavigationTypes.h"
#include "Chaos/AABB.h"
#include "Chaos/AABB.h"


#include <Runtime/AIModule/Classes/Navigation/PathFollowingComponent.h>
#include <Runtime/AIModule/Classes/Tasks/AITask.h>

#include "SVOAITask_MoveTo.generated.h"

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams( FSVOMoveTaskCompletedDelegate, TEnumAsByte< EPathFollowingResult::Type >, Result, AAIController *, AIController );

UCLASS()
class SVONAVIGATION_API USVOAITask_MoveTo : public UAITask
{
    GENERATED_BODY()

public:

    USVOAITask_MoveTo( const FObjectInitializer & object_initializer );

    bool WasMoveSuccessful() const;
    void PerformMove();
    void ConditionalPerformMove();
    void SetUp( AAIController * controller, const FAIMoveRequest & move_request );
    void SetContinuousGoalTracking( bool continuous_goal_tracking );
    void SetAsyncPathFinding( bool async_pathfinding );

    UFUNCTION( BlueprintCallable, Category = "AI|Tasks", meta = ( AdvancedDisplay = "acceptance_radius,stop_on_overlap,continuous_goal_tracking", DefaultToSelf = "Controller", BlueprintInternalUseOnly = "TRUE", DisplayName = "Move To Location or Actor" ) )
    static USVOAITask_MoveTo * AIMoveToInSVO( AAIController * controller, FVector goal_location, AActor * goal_actor = nullptr, float acceptance_radius = -1.f, bool async_pathfinding = true, EAIOptionFlag::Type stop_on_overlap = EAIOptionFlag::Default, bool lock_ai_logic = true, bool continuous_goal_tracking = false );

private:
    UPathFollowingComponent * GetPathFollowingComponent() const;
    void FinishMoveTask( EPathFollowingResult::Type result );
    void ResetObservers();
    void ResetTimers();
    void Activate() override;
    void OnDestroy( bool owner_finished ) override;
    void Pause() override;
    void Resume() override;
    void SetObservedPath( FNavPathSharedPtr path );
    void OnPathEvent( FNavigationPath * path, ENavPathEvent::Type path_event );
    void ConditionalUpdatePath();
    void OnRequestFinished( FAIRequestID request_id, const FPathFollowingResult & result );
    bool BuildPathfindingQuery( FSVOPathFindingQuery & path_finding_query, const FAIMoveRequest & move_request ) const;
    void FindPathForMoveRequest( const FAIMoveRequest & move_request, const FSVOPathFindingQuery & path_finding_query, FNavPathSharedPtr & path ) const;
    FPathFollowingRequestResult MoveTo( FNavPathSharedPtr * out_path, const FAIMoveRequest & move_request );
    FAIRequestID RequestMove( const FAIMoveRequest & move_request, FNavPathSharedPtr path ) const;

    /** parameters of move request */
    UPROPERTY()
    FAIMoveRequest MoveRequest;

    UPROPERTY( BlueprintAssignable )
    FGenericGameplayTaskDelegate OnRequestFailed;

    UPROPERTY( BlueprintAssignable )
    FSVOMoveTaskCompletedDelegate OnMoveFinishedDelegate;

    TEnumAsByte< EPathFollowingResult::Type > MoveResult;

    FAIRequestID MoveRequestID;
    FTimerHandle MoveRetryTimerHandle;
    FTimerHandle PathRetryTimerHandle;
    FDelegateHandle PathFinishDelegateHandle;
    FDelegateHandle PathUpdateDelegateHandle;

    uint8 UseContinuousTracking : 1;
    uint8 UseAsyncPathFinding : 1;
    FNavPathSharedPtr Path;
};

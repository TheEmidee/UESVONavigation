#pragma once

#include <BehaviorTree/Tasks/BTTask_BlackboardBase.h>
#include <CoreMinimal.h>

#include "SVOBTTask_MoveTo.generated.h"

class USVOAITask_MoveTo;

struct FSVOBTTaskMoveToMemory
{
    /** Move request ID */
    FAIRequestID MoveRequestID;

    FDelegateHandle BBObserverDelegateHandle;
    FVector PreviousGoalLocation;

    TWeakObjectPtr< USVOAITask_MoveTo > Task;

    uint8 bWaitingForPath : 1;
    uint8 bObserverCanFinishTask : 1;
};

UCLASS( config = Game )
class SVONAVIGATION_API USVOBTTask_MoveTo : public UBTTask_BlackboardBase
{
    GENERATED_BODY()

    USVOBTTask_MoveTo( const FObjectInitializer & object_initializer );

public:
    EBTNodeResult::Type ExecuteTask( UBehaviorTreeComponent & owner_component, uint8 * node_memory ) override;
    EBTNodeResult::Type AbortTask( UBehaviorTreeComponent & owner_component, uint8 * node_memory ) override;
    void OnTaskFinished( UBehaviorTreeComponent & owner_component, uint8 * node_memory, EBTNodeResult::Type task_result ) override;
    void TickTask( UBehaviorTreeComponent & owner_component, uint8 * node_memory, float delta_seconds ) override;
    uint16 GetInstanceMemorySize() const override;

    void OnGameplayTaskDeactivated( UGameplayTask & task ) override;
    void OnMessage( UBehaviorTreeComponent & owner_component, uint8 * node_memory, FName message, int32 request_id, bool success ) override;

    FString GetStaticDescription() const override;

#if WITH_EDITOR
    FName GetNodeIconName() const override;
#endif // WITH_EDITOR

private:

    EBlackboardNotificationResult OnBlackboardValueChange( const UBlackboardComponent & blackboard, FBlackboard::FKey change_key_id );
    EBTNodeResult::Type PerformMoveTask( UBehaviorTreeComponent & owner_component, uint8 * node_memory );

    /** prepares move task for activation */
    virtual USVOAITask_MoveTo * PrepareMoveTask( UBehaviorTreeComponent & owner_component, USVOAITask_MoveTo * existing_task, FAIMoveRequest & move_request );

    /** fixed distance added to threshold between AI and goal location in destination reach test */
    UPROPERTY( config, Category = Node, EditAnywhere, meta = ( ClampMin = "0.0", UIMin = "0.0" ) )
    float AcceptableRadius;

    /** if task is expected to react to changes to location represented by BB key
	*	this property can be used to tweak sensitivity of the mechanism. Value is
	*	recommended to be less then AcceptableRadius */
    UPROPERTY( Category = Blackboard, EditAnywhere, AdvancedDisplay, meta = ( ClampMin = "1", UIMin = "1", EditCondition = "bObserveBlackboardValue" ) )
    float ObservedBlackboardValueTolerance;

    /** if move goal in BB changes the move will be redirected to new location */
    UPROPERTY()
    uint32 bObserveBlackboardValue : 1;

    /** if set, path to goal actor will update itself when actor moves */
    UPROPERTY( Category = Node, EditAnywhere, AdvancedDisplay )
    uint32 bTrackMovingGoal : 1;

    /** if set, radius of AI's capsule will be added to threshold between AI and goal location in destination reach test  */
    UPROPERTY( Category = Node, EditAnywhere )
    uint32 bReachTestIncludesAgentRadius : 1;

    /** if set, radius of goal's capsule will be added to threshold between AI and goal location in destination reach test  */
    UPROPERTY( Category = Node, EditAnywhere )
    uint32 bReachTestIncludesGoalRadius : 1;

    /** if set, radius of AI's capsule will be added to threshold between AI and goal location in destination reach test  */
    UPROPERTY( Category = Node, EditAnywhere )
    bool bUseAsyncPathfinding;

    /** set automatically if move should use GameplayTasks */
    uint32 bUseGameplayTasks : 1;
};

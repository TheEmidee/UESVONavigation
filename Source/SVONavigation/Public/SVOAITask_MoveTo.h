#pragma once

#include <Runtime/AIModule/Classes/Navigation/PathFollowingComponent.h>
#include <Runtime/AIModule/Classes/Tasks/AITask.h>

#include "SVOAITask_MoveTo.generated.h"

UCLASS()
class SVONAVIGATION_API USVOAITask_MoveTo : public UAITask
{
    GENERATED_BODY()

public:

    USVOAITask_MoveTo( const FObjectInitializer & object_initializer );

    bool WasMoveSuccessful() const;
    void PerformMove();
    void ConditionalPerformMove();
    void SetUp( AAIController * controller, const FAIMoveRequest & move_request, bool use_async_pathfinding );

private:
    TEnumAsByte< EPathFollowingResult::Type > MoveResult;

    /** parameters of move request */
    UPROPERTY()
    FAIMoveRequest MoveRequest;

    FTimerHandle MoveRetryTimerHandle;
    bool UseAsyncPathfinding;
};

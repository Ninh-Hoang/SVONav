#pragma once

#include "SVONavType.h"
#include "SVONavDefines.h"

#include <Runtime/AIModule/Classes/Navigation/PathFollowingComponent.h>
#include <Runtime/AIModule/Classes/Tasks/AITask.h>
#include <Runtime/Core/Public/HAL/ThreadSafeBool.h>

#include "AITask_SVONavMoveTo.generated.h"

class AAIController;
class USVONavComponent;
struct FSVONavNavigationPath;

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FSVONavMoveTaskCompletedSignature, TEnumAsByte<EPathFollowingResult::Type>,
                                             Result, AAIController*, AIController);

UCLASS()
class SVONAV_API UAITask_SVONavMoveTo : public UAITask
{
	GENERATED_BODY()

public:
	UAITask_SVONavMoveTo(const FObjectInitializer& ObjectInitializer);

	/** tries to start move request and handles retry timer */
	void ConditionalPerformMove();

	/** prepare move task for activation */
	void SetUp(AAIController* Controller, const FAIMoveRequest& InMoveRequest, bool InUseAsyncPathfinding);

	EPathFollowingResult::Type GetMoveResult() const { return MoveResult; }
	bool WasMoveSuccessful() const { return MoveResult == EPathFollowingResult::Success; }

	UFUNCTION(BlueprintCallable, Category = "AI|Tasks",
		meta = (AdvancedDisplay =
			"AcceptanceRadius,StopOnOverlap,AcceptPartialPath,bUsePathfinding,bUseContinuosGoalTracking", DefaultToSelf
			= "Controller", BlueprintInternalUseOnly = "TRUE", DisplayName = "SVONav Move To Location or Actor"))
	static UAITask_SVONavMoveTo* SVONavAIMoveTo(AAIController* Controller, FVector GoalLocation,
	                                            bool InUseAsyncPathfinding, AActor* GoalActor = nullptr,
	                                            float AcceptanceRadius = -1.f,
	                                            EAIOptionFlag::Type StopOnOverlap = EAIOptionFlag::Default,
	                                            bool InCheckLineOfSight = true, bool InLockAILogic = true,
	                                            bool InUseContinuousGoalTracking = false);

	/** Allows custom move request tweaking. Note that all MoveRequest need to
	*	be performed before PerformMove is called. */
	FAIMoveRequest& GetMoveRequestRef() { return MoveRequest; }

	/** Switch task into continuous tracking mode: keep restarting move toward goal actor. Only pathfinding failure or external cancel will be able to stop this task. */
	void SetContinuousGoalTracking(bool InEnable);

	void TickTask(float DeltaTime) override;

protected:
	void LogPathHelper();

	FThreadSafeBool AsyncTaskComplete;
	bool UseAsyncPathfinding;

	UPROPERTY(BlueprintAssignable)
	FGenericGameplayTaskDelegate OnRequestFailed;

	UPROPERTY(BlueprintAssignable)
	FSVONavMoveTaskCompletedSignature OnMoveFinished;

	/** parameters of move request */
	UPROPERTY()
	FAIMoveRequest MoveRequest;

	bool CheckLineOfSight;

	/** handle of path following's OnMoveFinished delegate */
	FDelegateHandle PathFinishDelegateHandle;

	/** handle of path's update event delegate */
	FDelegateHandle PathUpdateDelegateHandle;

	/** handle of active ConditionalPerformMove timer  */
	FTimerHandle MoveRetryTimerHandle;

	/** handle of active ConditionalUpdatePath timer */
	FTimerHandle PathRetryTimerHandle;

	/** request ID of path following's request */
	FAIRequestID MoveRequestID;

	/** currently followed path */
	FNavPathSharedPtr Path;

	FSVONavPathSharedPtr SVONavPath;

	TEnumAsByte<EPathFollowingResult::Type> MoveResult;
	uint8 bUseContinuousTracking : 1;

	virtual void Activate() override;
	virtual void OnDestroy(bool InOwnerFinished) override;

	virtual void Pause() override;
	virtual void Resume() override;

	/** finish task */
	void FinishMoveTask(EPathFollowingResult::Type InResult);

	/** stores path and starts observing its events */
	void SetObservedPath(FNavPathSharedPtr InPath);

	//FPathFollowingRequestResult SVOResult;

	FSVONavPathfindingRequestResult SVOResult;

	USVONavComponent* SVONavComponent;

	void CheckPathPreConditions();

	void RequestPathSynchronous();
	void RequestPathAsync();

	void RequestMove();

	void HandleAsyncPathTaskComplete();

	void ResetPaths();

	/** remove all delegates */
	virtual void ResetObservers();

	/** remove all timers */
	virtual void ResetTimers();

	/** tries to update invalidated path and handles retry timer */
	void ConditionalUpdatePath();

	/** start move request */
	virtual void PerformMove();

	/** event from followed path */
	virtual void OnPathEvent(FNavigationPath* InPath, ENavPathEvent::Type Event);

	/** event from path following */
	virtual void OnRequestFinished(FAIRequestID RequestID, const FPathFollowingResult& Result);
};

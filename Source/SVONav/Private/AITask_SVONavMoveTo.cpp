#include "AITask_SVONavMoveTo.h"
#include "SVONav/SVONav.h"
#include "SVONavComponent.h"
#include "SVONavVolume.h"

#include <Runtime/AIModule/Classes/AIController.h>
#include <Runtime/AIModule/Classes/AIResources.h>
#include <Runtime/AIModule/Classes/AISystem.h>
#include <Runtime/Engine/Public/VisualLogger/VisualLogger.h>
#include <Runtime/GameplayTasks/Classes/GameplayTasksComponent.h>

UAITask_SVONavMoveTo::UAITask_SVONavMoveTo(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
	bIsPausable = true;
	MoveRequestID = FAIRequestID::InvalidRequest;

	MoveRequest.SetAcceptanceRadius(GET_AI_CONFIG_VAR(AcceptanceRadius));
	MoveRequest.SetReachTestIncludesAgentRadius(GET_AI_CONFIG_VAR(bFinishMoveOnGoalOverlap));
	MoveRequest.SetAllowPartialPath(GET_AI_CONFIG_VAR(bAcceptPartialPaths));
	MoveRequest.SetUsePathfinding(true);

	SVOResult.Code = ESVONavPathfindingRequestResult::Failed;

	AddRequiredResource(UAIResource_Movement::StaticClass());
	AddClaimedResource(UAIResource_Movement::StaticClass());

	MoveResult = EPathFollowingResult::Invalid;
	bUseContinuousTracking = false;

	Path = MakeShareable<FNavigationPath>(new FNavigationPath());
}

void UAITask_SVONavMoveTo::ConditionalPerformMove()
{
	if (MoveRequest.IsUsingPathfinding() && OwnerController && OwnerController->ShouldPostponePathUpdates())
	{
		UE_VLOG(GetGameplayTasksComponent(), LogGameplayTasks, Log, TEXT("%s> can't path right now, waiting..."),
		        *GetName());
		OwnerController->GetWorldTimerManager().SetTimer(MoveRetryTimerHandle, this,
		                                                 &UAITask_SVONavMoveTo::ConditionalPerformMove, 0.2f, false);
	}
	else
	{
		MoveRetryTimerHandle.Invalidate();
		PerformMove();
	}
}

void UAITask_SVONavMoveTo::SetUp(AAIController* Controller, const FAIMoveRequest& InMoveRequest,
                                 bool InUseAsyncPathfinding)
{
	OwnerController = Controller;
	MoveRequest = InMoveRequest;
	UseAsyncPathfinding = InUseAsyncPathfinding;
	bTickingTask = InUseAsyncPathfinding;

	// Fail if no nav component
	SVONavComponent = Cast<USVONavComponent>(GetOwnerActor()->FindComponentByClass(USVONavComponent::StaticClass()));
	if (!SVONavComponent)
	{
#if WITH_EDITOR
		UE_VLOG(this, VLogSVONav, Error, TEXT("SVONMoveTo request failed due missing SVONNavComponent"),
		        *MoveRequest.ToString());
		UE_LOG(LogSVONav, Error, TEXT("SVONMoveTo request failed due missing SVONNavComponent on the pawn"));
		return;
#endif
	}
	// Use the path instance from the navcomponent
	SVONavPath = SVONavComponent->GetPath();
}

UAITask_SVONavMoveTo* UAITask_SVONavMoveTo::SVONavAIMoveTo(AAIController* Controller, FVector InGoalLocation,
                                                           bool InUseAsyncPathfinding, AActor* InGoalActor,
                                                           float AcceptanceRadius, EAIOptionFlag::Type StopOnOverlap,
                                                           bool InCheckLineOfSight,
                                                           bool InLockAILogic, bool InUseContinuousGoalTracking)
{
	UAITask_SVONavMoveTo* MyTask = Controller
		                               ? UAITask::NewAITask<UAITask_SVONavMoveTo>(*Controller, EAITaskPriority::High)
		                               : nullptr;
	if (MyTask)
	{
		MyTask->UseAsyncPathfinding = InUseAsyncPathfinding;
		// We need to tick the task if we're using async, to check when results are back
		MyTask->bTickingTask = InUseAsyncPathfinding;

		FAIMoveRequest MoveReq;
		if (InGoalActor)
		{
			MoveReq.SetGoalActor(InGoalActor);
		}
		else
		{
			MoveReq.SetGoalLocation(InGoalLocation);
		}

		MoveReq.SetAcceptanceRadius(AcceptanceRadius);
		MoveReq.SetReachTestIncludesAgentRadius(
			FAISystem::PickAIOption(StopOnOverlap, MoveReq.IsReachTestIncludingAgentRadius()));
		if (Controller)
		{
			MoveReq.SetNavigationFilter(Controller->GetDefaultNavigationFilterClass());
		}

		MyTask->SetUp(Controller, MoveReq, InUseAsyncPathfinding);
		MyTask->SetContinuousGoalTracking(InUseContinuousGoalTracking);

		if (InLockAILogic)
		{
			MyTask->RequestAILogicLocking();
		}

		MyTask->CheckLineOfSight = InCheckLineOfSight;
	}

	return MyTask;
}

void UAITask_SVONavMoveTo::SetContinuousGoalTracking(bool InEnable)
{
	bUseContinuousTracking = InEnable;
}

void UAITask_SVONavMoveTo::TickTask(float DeltaTime)
{
	if (AsyncTaskComplete)
		HandleAsyncPathTaskComplete();
}

void UAITask_SVONavMoveTo::LogPathHelper()
{
#if WITH_EDITOR
#if ENABLE_VISUAL_LOG

	USVONavComponent* svonNavComponent = Cast<USVONavComponent>(
		GetOwnerActor()->GetComponentByClass(USVONavComponent::StaticClass()));
	if (!svonNavComponent)
		return;

	FVisualLogger& Vlog = FVisualLogger::Get();
	if (Vlog.IsRecording() &&
		SVONavPath.IsValid() && SVONavPath.Get()->GetPoints().Num())
	{
		FVisualLogEntry* Entry = Vlog.GetEntryToWrite(OwnerController->GetPawn(),
		                                              OwnerController->GetPawn()->GetWorld()->TimeSeconds);
		if (Entry)
		{
			for (int i = 0; i < SVONavPath->GetPoints().Num(); i++)
			{
				if (i == 0 || i == SVONavPath->GetPoints().Num() - 1)
					continue;

				const FSVONavPathPoint& Point = SVONavPath->GetPoints()[i];

				float Size = 0.f;

				if (Point.Layer == 0)
				{
					Size = svonNavComponent->GetCurrentVolume()->GetVoxelScale(0) * 0.25f;
				}
				else
				{
					Size = svonNavComponent->GetCurrentVolume()->GetVoxelScale(Point.Layer - 1);
				}


				UE_VLOG_BOX(OwnerController->GetPawn(), VLogSVONav, Verbose,
				            FBox(Point.Location + FVector(Size * 0.5f), Point.Location - FVector(Size * 0.5f)),
				            FColor::Black, TEXT_EMPTY);
			}
		}
	}
#endif // ENABLE_VISUAL_LOG
#endif // WITH_EDITOR
}

void UAITask_SVONavMoveTo::Activate()
{
	Super::Activate();
	UE_CVLOG(bUseContinuousTracking, GetGameplayTasksComponent(), LogGameplayTasks, Log,
	         TEXT("Continuous goal tracking requested, moving to: %s"),
	         MoveRequest.IsMoveToActorRequest() ? TEXT("actor => looping successful moves!") : TEXT(
		         "location => will NOT loop"));

	MoveRequestID = FAIRequestID::InvalidRequest;
	ConditionalPerformMove();
}

void UAITask_SVONavMoveTo::OnDestroy(bool InOwnerFinished)
{
	Super::OnDestroy(InOwnerFinished);

	ResetObservers();
	ResetTimers();

	if (MoveRequestID.IsValid())
	{
		UPathFollowingComponent* PFComp = OwnerController ? OwnerController->GetPathFollowingComponent() : nullptr;
		if (PFComp && PFComp->GetStatus() != EPathFollowingStatus::Idle)
		{
			PFComp->AbortMove(*this, FPathFollowingResultFlags::OwnerFinished, MoveRequestID);
		}
	}

	// clear the shared pointer now to make sure other systems
	// don't think this path is still being used
	Path = nullptr;
	SVONavPath = nullptr;
}

void UAITask_SVONavMoveTo::Pause()
{
	if (OwnerController && MoveRequestID.IsValid())
	{
		OwnerController->PauseMove(MoveRequestID);
	}

	ResetTimers();
	Super::Pause();
}

void UAITask_SVONavMoveTo::Resume()
{
	Super::Resume();

	if (!MoveRequestID.IsValid() || (OwnerController && !OwnerController->ResumeMove(MoveRequestID)))
	{
		UE_CVLOG(MoveRequestID.IsValid(), GetGameplayTasksComponent(), LogGameplayTasks, Log,
		         TEXT("%s> Resume move failed, starting new one."), *GetName());
		ConditionalPerformMove();
	}
}

void UAITask_SVONavMoveTo::FinishMoveTask(EPathFollowingResult::Type InResult)
{
	if (MoveRequestID.IsValid())
	{
		UPathFollowingComponent* PFComp = OwnerController ? OwnerController->GetPathFollowingComponent() : nullptr;
		if (PFComp && PFComp->GetStatus() != EPathFollowingStatus::Idle)
		{
			ResetObservers();
			PFComp->AbortMove(*this, FPathFollowingResultFlags::OwnerFinished, MoveRequestID);
		}
	}

	MoveResult = InResult;
	EndTask();

	if (InResult == EPathFollowingResult::Invalid)
	{
		OnRequestFailed.Broadcast();
	}
	else
	{
		OnMoveFinished.Broadcast(InResult, OwnerController);
	}
}

void UAITask_SVONavMoveTo::SetObservedPath(FNavPathSharedPtr InPath)
{
	if (PathUpdateDelegateHandle.IsValid() && Path.IsValid())
	{
		Path->RemoveObserver(PathUpdateDelegateHandle);
	}

	PathUpdateDelegateHandle.Reset();

	Path = InPath;
	if (Path.IsValid())
	{
		// disable auto repaths, it will be handled by move task to include ShouldPostponePathUpdates condition
		Path->EnableRecalculationOnInvalidation(false);
		PathUpdateDelegateHandle = Path->AddObserver(
			FNavigationPath::FPathObserverDelegate::FDelegate::CreateUObject(this, &UAITask_SVONavMoveTo::OnPathEvent));
	}
}

void UAITask_SVONavMoveTo::CheckPathPreConditions()
{
#if WITH_EDITOR
	UE_VLOG(this, VLogSVONav, Log, TEXT("SVONMoveTo: %s"), *MoveRequest.ToString());
#endif

	SVOResult.Code = ESVONavPathfindingRequestResult::Failed;

	// Fail if no nav component
	SVONavComponent = Cast<USVONavComponent>(GetOwnerActor()->GetComponentByClass(USVONavComponent::StaticClass()));
	if (!SVONavComponent)
	{
#if WITH_EDITOR
		UE_VLOG(this, VLogSVONav, Error, TEXT("SVONMoveTo request failed due missing SVONNavComponent on the pawn"),
		        *MoveRequest.ToString());
		UE_LOG(LogSVONav, Error, TEXT("SVONMoveTo request failed due missing SVONNavComponent on the pawn"));
		return;
#endif
	}

	if (MoveRequest.IsValid() == false)
	{
#if WITH_EDITOR
		UE_VLOG(this, VLogSVONav, Error,
		        TEXT(
			        "SVONMoveTo request failed due MoveRequest not being valid. Most probably desired Goal Actor not longer exists"
		        ), *MoveRequest.ToString());
		UE_LOG(LogSVONav, Error,
		       TEXT(
			       "SVONMoveTo request failed due MoveRequest not being valid. Most probably desired Goal Actor not longer exists"
		       ));
#endif
		return;
	}

	if (OwnerController->GetPathFollowingComponent() == nullptr)
	{
#if WITH_EDITOR
		UE_VLOG(this, VLogSVONav, Error, TEXT("SVONMoveTo request failed due missing PathFollowingComponent"));
		UE_LOG(LogSVONav, Error, TEXT("SVONMoveTo request failed due missing PathFollowingComponent"));
#endif
		return;
	}

	bool bCanRequestMove = true;
	bool bAlreadyAtGoal = false;

	if (!MoveRequest.IsMoveToActorRequest())
	{
		if (MoveRequest.GetGoalLocation().ContainsNaN() || FAISystem::IsValidLocation(MoveRequest.GetGoalLocation()) ==
			false)
		{
#if WITH_EDITOR
			UE_VLOG(this, VLogSVONav, Error, TEXT("SVONMoveTo: Destination is not valid! Goal(%s)"),
			        TEXT_AI_LOCATION(MoveRequest.GetGoalLocation()));
			UE_LOG(LogSVONav, Error, TEXT("SVONMoveTo: Destination is not valid! Goal(%s)"));
#endif
			bCanRequestMove = false;
		}

		bAlreadyAtGoal = bCanRequestMove && OwnerController->GetPathFollowingComponent()->HasReached(MoveRequest);
	}
	else
	{
		bAlreadyAtGoal = bCanRequestMove && OwnerController->GetPathFollowingComponent()->HasReached(MoveRequest);
	}

	if (bAlreadyAtGoal)
	{
#if WITH_EDITOR
		UE_VLOG(this, VLogSVONav, Log, TEXT("SVONMoveTo: already at goal!"));
		UE_LOG(LogSVONav, Log, TEXT("SVONMoveTo: already at goal!"));
#endif
		SVOResult.MoveId = OwnerController->GetPathFollowingComponent()->RequestMoveWithImmediateFinish(
			EPathFollowingResult::Success);
		SVOResult.Code = ESVONavPathfindingRequestResult::AlreadyAtGoal;
	}

	if (bCanRequestMove)
	{
		SVOResult.Code = ESVONavPathfindingRequestResult::ReadyToPath;
	}

	return;
}

void UAITask_SVONavMoveTo::RequestPathSynchronous()
{
	SVOResult.Code = ESVONavPathfindingRequestResult::Failed;

#if WITH_EDITOR
	UE_VLOG(this, VLogSVONav, Log, TEXT("SVONavMoveTo: Requesting Synchronous pathfinding!"));
	UE_LOG(LogSVONav, Log, TEXT("SVONavMoveTo: Requesting Synchronous pathfinding!"));
#endif

	ESVONavPathFindingCallResult CallResult;
	if (SVONavComponent->FindPathImmediate(SVONavComponent->GetPawnPosition(),
	                                       MoveRequest.IsMoveToActorRequest()
		                                       ? MoveRequest.GetGoalActor()->GetActorLocation()
		                                       : MoveRequest.GetGoalLocation(), CheckLineOfSight, &SVONavPath,
	                                       CallResult))
	{
		SVOResult.Code = ESVONavPathfindingRequestResult::Success;
	}
}

void UAITask_SVONavMoveTo::RequestPathAsync()
{
	SVOResult.Code = ESVONavPathfindingRequestResult::Failed;

	// Fail if no nav component
	USVONavComponent* NavComponent = Cast<USVONavComponent>(
		GetOwnerActor()->GetComponentByClass(USVONavComponent::StaticClass()));
	if (!NavComponent)
		return;

	AsyncTaskComplete = false;

	// Request the async path
	ESVONavPathFindingCallResult CallResult;
	NavComponent->FindPathAsync(SVONavComponent->GetPawnPosition(),
	                            MoveRequest.IsMoveToActorRequest()
		                            ? MoveRequest.GetGoalActor()->GetActorLocation()
		                            : MoveRequest.GetGoalLocation(),
	                            CheckLineOfSight,
	                            AsyncTaskComplete,
	                            &SVONavPath, CallResult);

	SVOResult.Code = ESVONavPathfindingRequestResult::Deferred;
}

void UAITask_SVONavMoveTo::RequestMove()
{
	SVOResult.Code = ESVONavPathfindingRequestResult::Failed;

	LogPathHelper();

	// Copy the SVO path into a regular path for now, until we implement our own path follower.
	SVONavPath->CreateNavPath(*Path);
	Path->MarkReady();

	UPathFollowingComponent* PFComp = OwnerController ? OwnerController->GetPathFollowingComponent() : nullptr;
	if (PFComp == nullptr)
	{
		FinishMoveTask(EPathFollowingResult::Invalid);
		return;
	}

	PathFinishDelegateHandle = PFComp->OnRequestFinished.AddUObject(this, &UAITask_SVONavMoveTo::OnRequestFinished);
	SetObservedPath(Path);

	const FAIRequestID RequestID = Path->IsValid()
		                               ? OwnerController->RequestMove(MoveRequest, Path)
		                               : FAIRequestID::InvalidRequest;
	if (RequestID.IsValid())
	{
#if WITH_EDITOR
		UE_VLOG(this, LogSVONav, Log, TEXT("SVONav Pathfinding successful, moving"));
		UE_LOG(VLogSVONav, Log, TEXT("SVONav Pathfinding successful, moving"));
#endif
		SVOResult.MoveId = RequestID;
		SVOResult.Code = ESVONavPathfindingRequestResult::Success;
	}

	if (SVOResult.Code == ESVONavPathfindingRequestResult::Failed)
	{
		SVOResult.MoveId = OwnerController->GetPathFollowingComponent()->RequestMoveWithImmediateFinish(
			EPathFollowingResult::Invalid);
	}
}

void UAITask_SVONavMoveTo::HandleAsyncPathTaskComplete()
{
	SVOResult.Code = ESVONavPathfindingRequestResult::Success;
	// Request the move
	RequestMove();
	// Flag that we've processed the task
	AsyncTaskComplete = false;
}

void UAITask_SVONavMoveTo::ResetPaths()
{
	if (Path.IsValid())
		Path->ResetForRepath();

	if (SVONavPath.IsValid())
		SVONavPath->Empty();
}

void UAITask_SVONavMoveTo::ResetObservers()
{
	if (Path.IsValid())
	{
		Path->DisableGoalActorObservation();
	}

	if (PathFinishDelegateHandle.IsValid())
	{
		UPathFollowingComponent* PFComp = OwnerController ? OwnerController->GetPathFollowingComponent() : nullptr;
		if (PFComp)
		{
			PFComp->OnRequestFinished.Remove(PathFinishDelegateHandle);
		}

		PathFinishDelegateHandle.Reset();
	}

	if (PathUpdateDelegateHandle.IsValid())
	{
		if (Path.IsValid())
		{
			Path->RemoveObserver(PathUpdateDelegateHandle);
		}

		PathUpdateDelegateHandle.Reset();
	}
}

void UAITask_SVONavMoveTo::ResetTimers()
{
	if (MoveRetryTimerHandle.IsValid())
	{
		if (OwnerController)
		{
			OwnerController->GetWorldTimerManager().ClearTimer(MoveRetryTimerHandle);
		}

		MoveRetryTimerHandle.Invalidate();
	}

	if (PathRetryTimerHandle.IsValid())
	{
		if (OwnerController)
		{
			OwnerController->GetWorldTimerManager().ClearTimer(PathRetryTimerHandle);
		}

		PathRetryTimerHandle.Invalidate();
	}
}

void UAITask_SVONavMoveTo::ConditionalUpdatePath()
{
	// mark this path as waiting for repath so that PathFollowingComponent doesn't abort the move while we 
	// micro manage repathing moment
	// note that this flag fill get cleared upon repathing end
	if (Path.IsValid())
	{
		Path->SetManualRepathWaiting(true);
	}

	if (MoveRequest.IsUsingPathfinding() && OwnerController && OwnerController->ShouldPostponePathUpdates())
	{
		UE_VLOG(GetGameplayTasksComponent(), LogGameplayTasks, Log, TEXT("%s> can't path right now, waiting..."),
		        *GetName());
		OwnerController->GetWorldTimerManager().SetTimer(PathRetryTimerHandle, this,
		                                                 &UAITask_SVONavMoveTo::ConditionalUpdatePath, 0.2f, false);
	}
	else
	{
		PathRetryTimerHandle.Invalidate();

		ANavigationData* NavData = Path.IsValid() ? Path->GetNavigationDataUsed() : nullptr;
		if (NavData)
		{
			NavData->RequestRePath(Path, ENavPathUpdateType::NavigationChanged);
		}
		else
		{
			UE_VLOG(GetGameplayTasksComponent(), LogGameplayTasks, Log, TEXT("%s> unable to repath, aborting!"),
			        *GetName());
			FinishMoveTask(EPathFollowingResult::Aborted);
		}
	}
}

void UAITask_SVONavMoveTo::PerformMove()
{
	// Prepare the move first (check for early out)
	CheckPathPreConditions();

	ResetObservers();
	ResetTimers();
	ResetPaths();

	if (SVOResult.Code == ESVONavPathfindingRequestResult::AlreadyAtGoal)
	{
		MoveRequestID = SVOResult.MoveId;
		OnRequestFinished(SVOResult.MoveId,
		                  FPathFollowingResult(EPathFollowingResult::Success,
		                                       FPathFollowingResultFlags::AlreadyAtGoal));
		return;
	}

	// If we're ready to path, then request the path
	if (SVOResult.Code == ESVONavPathfindingRequestResult::ReadyToPath)
	{
		UseAsyncPathfinding ? RequestPathAsync() : RequestPathSynchronous();

		switch (SVOResult.Code)
		{
		case ESVONavPathfindingRequestResult::Failed:
			FinishMoveTask(EPathFollowingResult::Invalid);
			break;
		case ESVONavPathfindingRequestResult::Success: // Synchronous pathfinding
			MoveRequestID = SVOResult.MoveId;
			if (IsFinished())
				UE_VLOG(GetGameplayTasksComponent(), LogGameplayTasks, Error, TEXT("%s> re-Activating Finished task!"),
			        *GetName());
			RequestMove(); // Start the move
			break;
		case ESVONavPathfindingRequestResult::Deferred: // Async...we're waiting on the task to return
			MoveRequestID = SVOResult.MoveId;
			AsyncTaskComplete = false;
			break;
		default:
			checkNoEntry();
			break;
		}
	}
}

void UAITask_SVONavMoveTo::OnPathEvent(FNavigationPath* InPath, ENavPathEvent::Type Event)
{
	const static UEnum* NavPathEventEnum = FindObject<UEnum>(ANY_PACKAGE, TEXT("ENavPathEvent"));
	UE_VLOG(GetGameplayTasksComponent(), LogGameplayTasks, Log, TEXT("%s> Path event: %s"), *GetName(),
	        *NavPathEventEnum->GetNameStringByValue(Event));

	switch (Event)
	{
	case ENavPathEvent::NewPath:
	case ENavPathEvent::UpdatedDueToGoalMoved:
	case ENavPathEvent::UpdatedDueToNavigationChanged:
		if (InPath && InPath->IsPartial() && !MoveRequest.IsUsingPartialPaths())
		{
			UE_VLOG(GetGameplayTasksComponent(), LogGameplayTasks, Log,
			        TEXT(">> partial path is not allowed, aborting"));
			UPathFollowingComponent::LogPathHelper(OwnerController, InPath, MoveRequest.GetGoalActor());
			FinishMoveTask(EPathFollowingResult::Aborted);
		}
#if ENABLE_VISUAL_LOG
		else if (!IsActive())
		{
			UPathFollowingComponent::LogPathHelper(OwnerController, InPath, MoveRequest.GetGoalActor());
		}
#endif // ENABLE_VISUAL_LOG
		break;

	case ENavPathEvent::Invalidated:
		ConditionalUpdatePath();
		break;

	case ENavPathEvent::Cleared:
	case ENavPathEvent::RePathFailed:
		UE_VLOG(GetGameplayTasksComponent(), LogGameplayTasks, Log, TEXT(">> no path, aborting!"));
		FinishMoveTask(EPathFollowingResult::Aborted);
		break;

	case ENavPathEvent::MetaPathUpdate:
	default:
		break;
	}
}

void UAITask_SVONavMoveTo::OnRequestFinished(FAIRequestID RequestID, const FPathFollowingResult& Result)
{
	if (RequestID == SVOResult.MoveId)
	{
		if (Result.HasFlag(FPathFollowingResultFlags::UserAbort) && Result.
			HasFlag(FPathFollowingResultFlags::NewRequest) && !Result.HasFlag(FPathFollowingResultFlags::ForcedScript))
		{
			UE_VLOG(GetGameplayTasksComponent(), LogGameplayTasks, Log,
			        TEXT("%s> ignoring OnRequestFinished, move was aborted by new request"), *GetName());
		}
		else
		{
			// reset request Id, FinishMoveTask doesn't need to update path following's state
			SVOResult.MoveId = FAIRequestID::InvalidRequest;

			if (bUseContinuousTracking && MoveRequest.IsMoveToActorRequest() && Result.IsSuccess())
			{
				UE_VLOG(GetGameplayTasksComponent(), LogGameplayTasks, Log,
				        TEXT("%s> received OnRequestFinished and goal tracking is active! Moving again in next tick"),
				        *GetName());
				GetWorld()->GetTimerManager().SetTimerForNextTick(this, &UAITask_SVONavMoveTo::PerformMove);
			}
			else
			{
				FinishMoveTask(Result.Code);
			}
		}
	}
	else if (IsActive())
	{
		UE_VLOG(GetGameplayTasksComponent(), LogGameplayTasks, Warning,
		        TEXT("%s> received OnRequestFinished with not matching RequestID!"), *GetName());
	}
}

#pragma once

#include "SVONavComponent.h"
#include "SVONavType.h"
#include "Async/Async.h"
#include "Async/AsyncWork.h"

class FSVONavFindPathTask : public FNonAbandonableTask
{
	friend class FAutoDeleteAsyncTask<FSVONavFindPathTask>;

public:
	FSVONavFindPathTask(
	ASVONavVolume& InVolume,
		UWorld* InWorld,
		const FSVONavLink InStartLink,
		const FSVONavLink InTargetLink,
		const FVector& InStartLocation,
		const FVector& InTargetLocation,
		FSVONavPathFindingConfig& InConfig,
		FSVONavPathSharedPtr* InPath,
		FThreadSafeBool& InCompleteFlag,
		/*const FFindPathTaskCompleteDynamicDelegate Complete,*/
		const bool InDrawDebug)
		: Volume(InVolume),
		  World(InWorld),
		  StartLink(InStartLink),
		  TargetLink(InTargetLink),
		  StartLocation(InStartLocation),
		  TargetLocation(InTargetLocation),
		  Config(InConfig),
		  Path(InPath),
		  CompleteFlag(InCompleteFlag),
		  //TaskComplete(Complete),
		  DrawDebug(InDrawDebug)
	{
	}

protected:
	ASVONavVolume& Volume;
	UWorld* World;

	FSVONavLink StartLink;
	FSVONavLink TargetLink;
	FVector StartLocation;
	FVector TargetLocation;
	FSVONavPathFindingConfig Config;

	FSVONavPathSharedPtr* Path;
	FThreadSafeBool& CompleteFlag;
	//FFindPathTaskCompleteDynamicDelegate TaskComplete;

	bool DrawDebug;

	void DoWork();

	// This next section of code needs to be here.  Not important as to why.

	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FSVONavFindPathTask, STATGROUP_ThreadPoolAsyncTasks);
	}
};

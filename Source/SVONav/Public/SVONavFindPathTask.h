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
		ASVONavVolume& Volume,
		UWorld* World,
		const FSVONavLink StartLink,
		const FSVONavLink TargetLink,
		const FVector& StartLocation,
		const FVector& TargetLocation,
		FSVONavPathFindingConfig& Config,
		FSVONavPath& Path,
		FThreadSafeBool& CompleteFlag,
		const FFindPathTaskCompleteDynamicDelegate Complete,
		bool DrawDebug)
		: Volume(Volume),
		  World(World),
		  StartLink(StartLink),
		  TargetLink(TargetLink),
		  StartLocation(StartLocation),
		  TargetLocation(TargetLocation),
		  Config(Config),
		  Path(Path),
		  CompleteFlag(CompleteFlag),
		  TaskComplete(Complete),
		  DrawDebug(DrawDebug)
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

	FSVONavPath& Path;
	FFindPathTaskCompleteDynamicDelegate TaskComplete;
	FThreadSafeBool& CompleteFlag;

	bool DrawDebug;

	void DoWork();

	// This next section of code needs to be here.  Not important as to why.

	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FSVONavFindPathTask, STATGROUP_ThreadPoolAsyncTasks);
	}
};

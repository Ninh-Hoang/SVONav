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
		ASVONavVolumeBase& InHieVolume,
		UWorld* InWorld,
		USVONavComponent* InNavComp,
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
		  HieVolume(InHieVolume),
		  World(InWorld),
		  NavComp(InNavComp),
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
	ASVONavVolumeBase& HieVolume;
	UWorld* World;
	USVONavComponent* NavComp;

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

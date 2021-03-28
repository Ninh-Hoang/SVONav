#pragma once

#include "SVONavVolume.h"
#include "Async/Async.h"
#include "Async/AsyncWork.h"

class FSVONavUpdateOctreeTask
{
friend class FAutoDeleteAsyncTask<FSVONavUpdateOctreeTask>;

public:
	FSVONavUpdateOctreeTask(
		ASVONavVolume* Volume,
		const FSVONavUpdateOctreeDelegate Complete) :
		Volume(Volume),
		TaskComplete(Complete){}

protected:
	ASVONavVolume* Volume;
	FSVONavUpdateOctreeDelegate TaskComplete;

	void DoWork() const {
		Volume->UpdateOctree();
		TaskComplete.Execute();
	}

	FORCEINLINE TStatId GetStatId() const
	{
		RETURN_QUICK_DECLARE_CYCLE_STAT(FSVONavUpdateOctreeTask, STATGROUP_ThreadPoolAsyncTasks);
	}
	
};

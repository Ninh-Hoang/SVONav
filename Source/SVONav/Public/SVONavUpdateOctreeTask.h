#pragma once

#include "SVONavVolumeBase.h"
#include "Async/Async.h"
#include "Async/AsyncWork.h"

class FSVONavUpdateOctreeTask : public FNonAbandonableTask
{
friend class FAutoDeleteAsyncTask<FSVONavUpdateOctreeTask>;

public:
	FSVONavUpdateOctreeTask(
		ASVONavVolumeBase* Volume,
		const FSVONavUpdateOctreeDelegate Complete) :
		Volume(Volume),
		TaskComplete(Complete){}

protected:
	ASVONavVolumeBase* Volume;
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

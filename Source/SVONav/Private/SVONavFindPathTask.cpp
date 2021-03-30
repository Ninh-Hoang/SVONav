#include "SVONavFindPathTask.h"

#include "SVONavPathFinder.h"

void FSVONavFindPathTask::DoWork()
{
	SVONavPathFinder PathFinder(World, NavComp, Volume, Config);
	// Run the path pruning, smoothing and debug draw back on the game thread

	int Result = PathFinder.FindPath(StartLink, TargetLink, StartLocation, TargetLocation, Config, Path);

	/*AsyncTask(ENamedThreads::GameThread, [=]()
	{*/
	//PathFinder.ApplyPathPruning(Path, Config);
	//PathFinder.ApplyPathSmoothing(Path, Config);

#if WITH_EDITOR
	if (DrawDebug)
	{
		PathFinder.DrawDebug(World, Volume, Path);
	}
#endif
	CompleteFlag = true;
	//});
}

#pragma once

#include "SVONavType.h"

class ASVONavVolume;

class SVONAV_API SVONavPathFinder
{
public:
	SVONavPathFinder(UWorld* InWorld, ASVONavVolume& InNavVolume, FSVONavPathFindingConfig& InConfig)
		: World(InWorld),
		  SVOVolume(InNavVolume),
		  Config(InConfig)
	{
	};

	~SVONavPathFinder()
	{
	};

	/* Performs an A* search from start to target navlink */
	int FindPath(const FSVONavLink& StartLink,
	             const FSVONavLink& TargetLink,
	             const FVector& StartLocation,
	             const FVector& TargetLocation,
	             FSVONavPathFindingConfig Config,
	             FSVONavPathSharedPtr* Path
	);

	void ApplyPathPruning(FSVONavPathSharedPtr* InPath, const FSVONavPathFindingConfig InConfig) const;
	void ApplyPathLineOfSight(FSVONavPathSharedPtr* InPath, AActor* Target, float MinimumDistance) const;
	static void ApplyPathSmoothing(FSVONavPathSharedPtr* InPath, FSVONavPathFindingConfig Config);

#if WITH_EDITOR
	void RequestNavPathDebugDraw(const FSVONavPathSharedPtr* InPath) const;

	void DrawDebug(UWorld* World, const ASVONavVolume& Volume, FSVONavPathSharedPtr* InPath) const;
#endif
	
private:
	// Initialise
	TSet<FSVONavLink> OpenSet;
	TSet<FSVONavLink> ClosedSet;
	TMap<FSVONavLink, FSVONavLink> Parent;
	TMap<FSVONavLink, float> GScore;
	TMap<FSVONavLink, float> FScore;

	FSVONavLink StartLink = FSVONavLink();
	FSVONavLink CurrentLink = FSVONavLink();
	FSVONavLink TargetLink = FSVONavLink();
	
	UWorld* World;
	ASVONavVolume& SVOVolume;
	FSVONavPathFindingConfig& Config;

	/* A* heuristic calculation */
	float HeuristicScore(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink);

	/* Distance between two links */
	float GetCost(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink);

	void ProcessLink(const FSVONavLink& NeighbourLink);

	/* Constructs the path by navigating back through our CameFrom map */
	void BuildPath(TMap<FSVONavLink, FSVONavLink>& InParent, FSVONavLink InCurrentLink, const FVector& InStartLocation,
	               const FVector& InTargetLocation, FSVONavPathSharedPtr* InPath);
};

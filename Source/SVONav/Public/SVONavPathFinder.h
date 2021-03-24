#pragma once

#include "SVONavType.h"

class ASVONavVolume;

class SVONAV_API SVONavPathFinder
{
public:
	SVONavPathFinder(UWorld* InWorld, ASVONavVolume& NavVolume, FSVONavPathFindingConfig& Config)
		: SVOVolume(NavVolume)
		  , Config(Config)
		  , World(InWorld)
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
	             FSVONavPath& Path
	);

	void ApplyPathPruning(FSVONavPath& Path, const FSVONavPathFindingConfig Config) const;
	void ApplyPathLineOfSight(FSVONavPath& Path, AActor* Target, float MinimumDistance) const;
	static void ApplyPathSmoothing(FSVONavPath& Path, FSVONavPathFindingConfig Config);

	void RequestNavPathDebugDraw(const FSVONavPath Path) const;

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

	ASVONavVolume& SVOVolume;
	FSVONavPathFindingConfig& Config;

	UWorld* World;

	/* A* heuristic calculation */
	float HeuristicScore(const FSVONavLink& StartLink, const FSVONavLink& TargetLink);

	/* Distance between two links */
	float GetCost(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink);

	void ProcessLink(const FSVONavLink& NeighbourLink);

	/* Constructs the path by navigating back through our CameFrom map */
	void BuildPath(TMap<FSVONavLink, FSVONavLink>& InParent, FSVONavLink InCurrentLink, const FVector& InStartLocation, const FVector& InTargetLocation, FSVONavPath& InPath);
};

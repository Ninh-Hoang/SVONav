#pragma once

#include "SVONavComponent.h"
#include "SVONavType.h"

class ASVONavVolume;

class SVONAV_API SVONavPathFinder
{
public:
	SVONavPathFinder(UWorld* InWorld,
	                 USVONavComponent* InNavComp,
	                 ASVONavVolume& InNavVolume,
	                 ASVONavVolumeBase& InHieVolume,
	                 FSVONavPathFindingConfig& InConfig)
		: World(InWorld),
		  NavComp(InNavComp),
		  SVOVolume(InNavVolume),
		  HieVolume(InHieVolume),
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

	void DrawDebug(UWorld* World, const ASVONavVolumeBase& Volume, FSVONavPathSharedPtr* InPath) const;
#endif

private:
	// Initialise
	TSet<FSVONavLink> TargetSet;
	TSet<FSVONavLink> OpenSet;
	TSet<FSVONavLink> ClosedSet;
	TMap<FSVONavLink, FSVONavLink> Parent;
	TMap<FSVONavLink, float> GScore;
	TMap<FSVONavLink, float> FScore;

	FSVONavLink StartLink = FSVONavLink();
	FSVONavLink CurrentLink = FSVONavLink();
	FSVONavLink TargetLink = FSVONavLink();

	UWorld* World;
	USVONavComponent* NavComp;
	ASVONavVolume& SVOVolume;
	ASVONavVolumeBase& HieVolume;
	FSVONavPathFindingConfig& Config;

	/* A* heuristic calculation */
	float HeuristicScore(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink);

	float HeuristicScoreHie(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink);

	/* Distance between two links */
	float GetCost(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink);
	float GetCostHie(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink);

	void ProcessLink(const FSVONavLink& NeighbourLink);
	void ProcessLinkHie(const FSVONavLink& NeighbourLink);

	int FindPathHierarchical(const FSVONavLink& StartLink,
	                         const FSVONavLink& TargetLink,
	                         const FVector& StartLocation,
	                         const FVector& TargetLocation,
	                         FSVONavPathFindingConfig Config,
	                         FSVONavPathSharedPtr* Path
	);

	int FindPathAStar(const FSVONavLink& StartLink,
	                  const FSVONavLink& TargetLink,
	                  const FVector& StartLocation,
	                  const FVector& TargetLocation,
	                  FSVONavPathFindingConfig Config,
	                  FSVONavPathSharedPtr* Path
	);

	int FindPathTesting(const FSVONavLink& StartLink,
	                    const FSVONavLink& TargetLink,
	                    const FVector& StartLocation,
	                    const FVector& TargetLocation,
	                    FSVONavPathFindingConfig Config,
	                    FSVONavPathSharedPtr* Path
	);

	/* Constructs the path by navigating back through our CameFrom map */
	void BuildPath(TMap<FSVONavLink, FSVONavLink>& InParent, FSVONavLink InCurrentLink, const FVector& InStartLocation,
	               const FVector& InTargetLocation, FSVONavPathSharedPtr* InPath);

	void BuildHierarchicalPath(TMap<FSVONavLink, FSVONavLink>& InParent,
	                           FSVONavLink TopStartLink,
	                           FSVONavLink TopTargetLink,
	                           FSVONavLink InStartLink,
	                           FSVONavLink InTargetLink,
	                           FSVONavLink InCurrentLink,
	                           const FVector& InStartLocation,
	                           const FVector& InTargetLocation, FSVONavPathSharedPtr* InPath);
	
	void RollHierarchicalPath(TMap<FSVONavLink, FSVONavLink>& InParent,
                               FSVONavLink InCurrentLink,
                               FSVONavLink InStartLink,
                               FSVONavLink InTargetLink,
                               const FVector& StartLocation,
								const FVector& TargetLocation,
                               FSVONavPathSharedPtr* InPath,
                               int32& PointIndex);

	int RefineHierarchicalPath(FSVONavLink InStartLink,
	                            FSVONavLink InTargetLink,
	                            FSVONavPathSharedPtr* InPath,
	                            int32 RefineIndex);


	void BuildPathHie(TMap<FSVONavLink, FSVONavLink>& InParent, FSVONavLink InCurrentLink,
	                  const FVector& InStartLocation,
	                  const FVector& InTargetLocation, FSVONavPathSharedPtr* InPath);
};

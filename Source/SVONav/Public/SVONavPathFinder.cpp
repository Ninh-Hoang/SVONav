#include "SVONavPathFinder.h"

#include "SVONavVolume.h"

int SVONavPathFinder::FindPath(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink,
                               const FVector& InStartLocation, const FVector& InTargetLocation,
                               FSVONavPathFindingConfig InConfig, FSVONavPath& InPath)
{
	//init
	OpenSet.Empty();
	ClosedSet.Empty();
	Parent.Empty();
	FScore.Empty();
	GScore.Empty();
	CurrentLink = FSVONavLink();
	InPath.Empty();

	StartLink = InStartLink;
	TargetLink = InTargetLink;

	OpenSet.Add(StartLink);
	Parent.Add(StartLink, StartLink);
	FScore.Add(StartLink, 0.f);
	GScore.Add(StartLink, HeuristicScore(StartLink, TargetLink));

	//iteration
	int Iterations = 0;

	while (OpenSet.Num() > 0)
	{
		float LowestScore = FLT_MAX;
		for (FSVONavLink& Link : OpenSet)
		{
			if (!FScore.Contains(Link) || FScore[Link] < LowestScore)
			{
				LowestScore = FScore[Link];
				CurrentLink = Link;
			}
		}
		OpenSet.Remove(CurrentLink);
		ClosedSet.Add(CurrentLink);

		if (CurrentLink.NodeIndex == TargetLink.NodeIndex)
		{
			BuildPath(Parent, CurrentLink, InStartLocation, InTargetLocation, InPath);
			return 1;
		}

		const FSVONavNode& CurrentNode = SVOVolume.GetNode(CurrentLink);
		TArray<FSVONavLink> NeighbourLinks;

		if (CurrentLink.GetLayerIndex() == 0 && CurrentNode.FirstChild.IsValid())
		{
			SVOVolume.GetNeighbourLeaves(CurrentLink, NeighbourLinks);
		}
		else
		{
			SVOVolume.GetNeighbourLeaves(CurrentLink, NeighbourLinks);
		}

		for (const FSVONavLink& Neighbour : NeighbourLinks)
		{
			ProcessLink(Neighbour);
		}

		Iterations++;
	}

#if WITH_EDITOR
	UE_LOG(LogTemp, Display, TEXT("Pathfinding failed, iterations : %i"), Iterations);
#endif
}

void SVONavPathFinder::ApplyPathPruning(FSVONavPath& Path, const FSVONavPathFindingConfig Config) const
{
}

void SVONavPathFinder::ApplyPathLineOfSight(FSVONavPath& Path, AActor* Target, float MinimumDistance) const
{
}

void SVONavPathFinder::ApplyPathSmoothing(FSVONavPath& Path, FSVONavPathFindingConfig Config)
{
}



float SVONavPathFinder::HeuristicScore(const FSVONavLink& StartLink, const FSVONavLink& TargetLink)
{
	float Score = 0.f;
	FVector StartLocation, TargetLocation;

	SVOVolume.GetLinkLocation(StartLink, StartLocation);
	SVOVolume.GetLinkLocation(TargetLink, TargetLocation);

	switch (Config.Heuristic)
	{
	case ESVONavHeuristic::Manhattan:
		Score = FMath::Abs(TargetLocation.X - StartLocation.X) + FMath::Abs(TargetLocation.Y - StartLocation.Y) +
			FMath::Abs(
				TargetLocation.Z - StartLocation.Z);
		break;
	case ESVONavHeuristic::Euclidean:
	default:
		Score = (StartLocation - TargetLocation).Size();
		break;
	}

	Score *= (1.0f - (static_cast<float>(TargetLink.GetLayerIndex()) / static_cast<float>(SVOVolume.NumLayers)) * Config
		.NodeSizePreference);
	return Score;
}

float SVONavPathFinder::GetCost(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink)
{
	float Cost = 1.f;

	// Unit cost implementation
	if (Config.UseUnitCost)
	{
		Cost = Config.UnitCost;
	}
	else
	{
		FVector StartPos(0.f), TargetPos(0.f);
		const FSVONavNode& StartNode = SVOVolume.GetNode(InStartLink);
		const FSVONavNode& EndNode = SVOVolume.GetNode(InTargetLink);
		SVOVolume.GetLinkLocation(InStartLink, StartPos);
		SVOVolume.GetLinkLocation(InTargetLink, TargetPos);
		Cost -= static_cast<float>(InTargetLink.GetLayerIndex()) / static_cast<float>(SVOVolume.NumLayers) * Config.
			NodeSizePreference;
		if (Config.Heuristic == ESVONavHeuristic::Euclidean) {
			Cost *= (StartPos - TargetPos).Size();
		}
	}

	return Cost;
}

void SVONavPathFinder::ProcessLink(const FSVONavLink& NeighbourLink)
{
	if (NeighbourLink.IsValid())
	{
		if (ClosedSet.Contains(NeighbourLink))
			return;

		if (!OpenSet.Contains(NeighbourLink))
		{
			OpenSet.Add(NeighbourLink);
		}

		float MaxGScore = FLT_MAX;
		if (GScore.Contains(CurrentLink))
			MaxGScore = GScore[CurrentLink] + GetCost(CurrentLink, NeighbourLink);
		else
			GScore.Add(CurrentLink, FLT_MAX);

		if (MaxGScore >= (GScore.Contains(NeighbourLink) ? GScore[NeighbourLink] : FLT_MAX))
			return;

		Parent.Add(NeighbourLink, CurrentLink);
		GScore.Add(NeighbourLink, MaxGScore);
		FScore.Add(NeighbourLink,
		           GScore[NeighbourLink] + (Config.EstimateWeight * HeuristicScore(NeighbourLink, TargetLink)));
	}
}

void SVONavPathFinder::BuildPath(TMap<FSVONavLink, FSVONavLink>& InParent, FSVONavLink InCurrentLink,
                                 const FVector& InStartLocation, const FVector& InTargetLocation, FSVONavPath& InPath)
{
	FSVONavPathPoint PathPoint;

	while (InParent.Contains(InCurrentLink) && !(InCurrentLink == InParent[InCurrentLink]))
	{
		InCurrentLink = InParent[InCurrentLink];
		SVOVolume.GetLinkLocation(InCurrentLink, PathPoint.Location);
		InPath.Points.Insert(PathPoint, 0);
		const FSVONavNode& Node = SVOVolume.GetNode(InCurrentLink);
		if (InCurrentLink.GetLayerIndex() == 0)
		{
			if (!Node.HasChildren()) InPath.Points[0].Layer = 1;
			else InPath.Points[0].Layer = 0;
		}
		else
		{
			InPath.Points[0].Layer = InCurrentLink.GetLayerIndex() + 1;
		}
	}

	if (InPath.Points.Num() > 1)
	{
		InPath.Points[0].Location = InStartLocation;
		InPath.Points[InPath.Points.Num() - 1].Location = InTargetLocation;
	}
	else
	{
		if (InPath.Points.Num() == 0) InPath.Points.Emplace();
		InPath.Points[0].Location = InTargetLocation;
		InPath.Points.Emplace(InStartLocation, StartLink.GetLayerIndex());
	}
}

#if WITH_EDITOR

void SVONavPathFinder::RequestNavPathDebugDraw(const FSVONavPath Path) const
{
	if (!World || !IsValid(&SVOVolume) || Path.Points.Num() < 2) return;
	FSVONavDebugPath DebugPath;
	for (auto& Point: Path.Points) DebugPath.Points.Add(Point.Location);
	//hardcore debug visual
	DebugPath.Color = FColor(0, 255, 255);
	DebugPath.LineScale = 10.0;
	
	SVOVolume.AddDebugNavPath(DebugPath);
}

#endif
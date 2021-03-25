#include "SVONavPathFinder.h"

#include "DrawDebugHelpers.h"
#include "SVONav/SVONav.h"
#include "SVONavVolume.h"

int SVONavPathFinder::FindPath(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink,
                               const FVector& InStartLocation, const FVector& InTargetLocation,
                               FSVONavPathFindingConfig InConfig, FSVONavPathSharedPtr* InPath)
{
	//init
	OpenSet.Empty();
	ClosedSet.Empty();
	Parent.Empty();
	FScore.Empty();
	GScore.Empty();
	CurrentLink = FSVONavLink();
	InPath->Get()->Empty();

	StartLink = InStartLink;
	TargetLink = InTargetLink;

	OpenSet.Add(InStartLink);
	Parent.Add(InStartLink, InStartLink);
	FScore.Add(InStartLink, 0.f);
	GScore.Add(InStartLink, HeuristicScore(InStartLink, TargetLink));

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
#if WITH_EDITOR
			UE_LOG(LogSVONav, Display, TEXT("Pathfinding complete, iterations : %i"), Iterations);
			
#endif
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
	UE_LOG(LogSVONav, Display, TEXT("Pathfinding failed, iterations : %i"), Iterations);
#endif
	return 0;
}

void SVONavPathFinder::ApplyPathPruning(FSVONavPathSharedPtr* InPath, const FSVONavPathFindingConfig InConfig) const
{
}

void SVONavPathFinder::ApplyPathLineOfSight(FSVONavPathSharedPtr* InPath, AActor* Target, float MinimumDistance) const
{
}

void SVONavPathFinder::ApplyPathSmoothing(FSVONavPathSharedPtr* InPath, FSVONavPathFindingConfig InConfig)
{
}


float SVONavPathFinder::HeuristicScore(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink)
{
	float Score = 0.f;
	FVector StartLocation, TargetLocation;

	SVOVolume.GetLinkLocation(InStartLink, StartLocation);
	SVOVolume.GetLinkLocation(InTargetLink, TargetLocation);

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

	Score *= (1.0f - (static_cast<float>(InTargetLink.GetLayerIndex()) / static_cast<float>(SVOVolume.NumLayers)) * Config
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
		SVOVolume.GetLinkLocation(InStartLink, StartPos);
		SVOVolume.GetLinkLocation(InTargetLink, TargetPos);
		Cost -= static_cast<float>(InTargetLink.GetLayerIndex()) / static_cast<float>(SVOVolume.NumLayers) * Config.
			NodeSizePreference;
		if (Config.Heuristic == ESVONavHeuristic::Euclidean)
		{
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
                                 const FVector& InStartLocation, const FVector& InTargetLocation,
                                 FSVONavPathSharedPtr* InPath)
{
	FSVONavPathPoint Location;

	TArray<FSVONavPathPoint> Points;

	if (!InPath || !InPath->IsValid())
		return;

	while (InParent.Contains(InCurrentLink) && !(InCurrentLink == InParent[InCurrentLink]))
	{
		InCurrentLink = InParent[InCurrentLink];
		SVOVolume.GetLinkLocation(InCurrentLink, Location.Location);
		Points.Add(Location);
		const FSVONavNode& node = SVOVolume.GetNode(InCurrentLink);
		// This is rank. I really should sort the layers out
		if (InCurrentLink.GetLayerIndex() == 0)
		{
			if (!node.HasChildren())
				Points[Points.Num() - 1].Layer = 1;
			else
				Points[Points.Num() - 1].Layer = 0;
		}
		else
		{
			Points[Points.Num() - 1].Layer = InCurrentLink.GetLayerIndex() + 1;
		}
	}
	if (Points.Num() > 1)
	{
		Points[0].Location = InTargetLocation;
		Points[Points.Num() - 1].Location = InStartLocation;
	}
	else // If start and end are in the same voxel, just use the start and target positions.
	{
		if (Points.Num() == 0)
			Points.Emplace();

		Points[0].Location = InTargetLocation;
		Points.Emplace(InStartLocation, StartLink.GetLayerIndex());
	}

	InPath->Get()->SetPoints(Points);
}

#if WITH_EDITOR

void SVONavPathFinder::DrawDebug(UWorld* InWorld, const ASVONavVolume& Volume, FSVONavPathSharedPtr* InPath) const
{
	TArray<FSVONavPathPoint> Points = InPath->Get()->GetPoints();
	for (int i = 0; i < Points.Num(); i++)
	{
		FSVONavPathPoint& Point = Points[i];

		if (i < Points.Num() - 1)
		{
			FVector OffSet(0.f);
			//if (i == 0)
			//offSet.Z -= 300.f;
			float Size = Point.Layer == 0 ? Volume.GetVoxelScale(Point.Layer) * 0.25f : Volume.GetVoxelScale(Point.Layer) * 0.5f;

			//DrawDebugBox(InWorld, Point.Location, FVector(Size), Volume.GetLayerColour(Point.Layer), true, -1.f, 0, 10.f);

			//DrawDebugSphere(InWorld, Point.Location + OffSet, 30.f, 20, FColor::Cyan, true, -1.f, 0, 20.f);

			DrawDebugLine(InWorld, Point.Location + OffSet, Points[i+1].Location, FColor::Cyan, true, -1.f, 0, 20.f);
		}
	}
}

void SVONavPathFinder::RequestNavPathDebugDraw(const FSVONavPathSharedPtr* InPath) const
{
	if (!World || !IsValid(&SVOVolume) || InPath->Get()->Points.Num() < 2) return;
	FSVONavDebugPath DebugPath;
	for (auto& Point : InPath->Get()->Points) DebugPath.Points.Add(Point.Location);
	//hardcore debug visual
	DebugPath.Color = FColor(0, 255, 255);
	DebugPath.LineScale = 10.0;

	SVOVolume.AddDebugNavPath(DebugPath);
}

#endif

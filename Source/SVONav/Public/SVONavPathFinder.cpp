#include "SVONavPathFinder.h"

#include "DrawDebugHelpers.h"
#include "SVONav/SVONav.h"
#include "SVONavVolume.h"

#include "chrono"
using namespace std::chrono;

int SVONavPathFinder::FindPath(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink,
                               const FVector& InStartLocation, const FVector& InTargetLocation,
                               FSVONavPathFindingConfig InConfig, FSVONavPathSharedPtr* InPath)
{
#if WITH_EDITOR
	const auto StartTime = high_resolution_clock::now();
#endif
	int Result;
	switch (Config.Algorithm)
	{
		case ESVONavAlgorithm::GreedyAStar:
			Result = FindPathAStar(InStartLink, InTargetLink, InStartLocation, InTargetLocation, InConfig, InPath);
		break;
		case ESVONavAlgorithm::Testing:
		default:
			Result = FindPathTesting(InStartLink, InTargetLink, InStartLocation, InTargetLocation, InConfig, InPath);
		break;
	}
#if WITH_EDITOR
	UE_LOG(LogSVONav, Display, TEXT("Pathfinding complete, iterations : %i"), Result);

	const float Duration = std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - StartTime).
        count() / 1000.0f;
	UE_LOG(LogSVONav, Display, TEXT("Path finding Time : %f"), Duration);
#endif

	if(Result!=0)
	{
		ApplyPathPruning(InPath, InConfig);
		ApplyPathSmoothing(InPath, InConfig);
	}
	
	return Result;
}

int SVONavPathFinder::FindPathTesting(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink,
	const FVector& InStartLocation, const FVector& InTargetLocation, FSVONavPathFindingConfig InConfig,
	FSVONavPathSharedPtr* InPath)
{
	// Initialise
	TMap<FSVONavLink, float> G;
	TMap<FSVONavLink, float> F;
	FSVONavLink CurrentEdge = FSVONavLink();
	InPath->Get()->Empty();

	// Greedy A*
	OpenSet.Add(InStartLink);
	Parent.Add(InStartLink, InStartLink);
	G.Add(InStartLink, 0);
	F.Add(InStartLink, HeuristicScore(InStartLink, InTargetLink));
	int32 I = 0;
	while (OpenSet.Num() > 0) {
		float LowestScore = FLT_MAX;
		for (FSVONavLink& Edge : OpenSet) {
			if (!F.Contains(Edge) || F[Edge] < LowestScore) {
				LowestScore = F[Edge];
				CurrentEdge = Edge;
			}
		}
		OpenSet.Remove(CurrentEdge);
		ClosedSet.Add(CurrentEdge);

		if (CurrentEdge.NodeIndex == InTargetLink.NodeIndex) {
			FSVONavPathPoint PathPoint;

			while (Parent.Contains(CurrentEdge) && !(CurrentEdge == Parent[CurrentEdge])) {
				CurrentEdge = Parent[CurrentEdge];
				SVOVolume.GetLinkLocation(CurrentEdge, PathPoint.Location);
				InPath->Get()->Points.Insert(PathPoint, 0);
				const FSVONavNode& Node = SVOVolume.GetNode(CurrentEdge);
				if (CurrentEdge.GetLayerIndex() == 0) {
					if (!Node.HasChildren()) InPath->Get()->Points[0].Layer = 1;
					else InPath->Get()->Points[0].Layer = 0;
				} else {
					InPath->Get()->Points[0].Layer = CurrentEdge.GetLayerIndex() + 1;
				}
			}
			
			if (InPath->Get()->Points.Num() > 1) {
				InPath->Get()->Points[0].Location = InStartLocation;
				InPath->Get()->Points[InPath->Get()->Points.Num() - 1].Location = InTargetLocation;
				
			} else {
				if (InPath->Get()->Points.Num() == 0) InPath->Get()->Points.Emplace();
				InPath->Get()->Points[0].Location = InTargetLocation;
				InPath->Get()->Points.Emplace(InStartLocation, InStartLink.GetLayerIndex());
			}
			
			return I;
		}

		const FSVONavNode& CurrentNode = SVOVolume.GetNode(CurrentEdge);
		TArray<FSVONavLink> AdjacentEdges;

		if (CurrentEdge.GetLayerIndex() == 0 && CurrentNode.FirstChild.IsValid()) {
			SVOVolume.GetNeighbourLeaves(CurrentEdge, AdjacentEdges);
		} else {
			SVOVolume.GetNeighbourLinks(CurrentEdge, AdjacentEdges);
		}

		for (auto& AdjacentEdge : AdjacentEdges) {
			if (AdjacentEdge.IsValid()) {
				if (ClosedSet.Contains(AdjacentEdge)) {
					continue;
				}
				if (!OpenSet.Contains(AdjacentEdge)) {
					OpenSet.Add(AdjacentEdge);
				}

				float MinGScore = FLT_MAX;
				if (G.Contains(CurrentEdge))
				{
					FVector CurrentLocation(0.f), AdjacentLocation(0.f);
					SVOVolume.GetLinkLocation(CurrentEdge, CurrentLocation);
					SVOVolume.GetLinkLocation(AdjacentEdge, AdjacentLocation);
					float Cost = 0.0f;
					
					Cost -= static_cast<float>(InTargetLink.GetLayerIndex()) / static_cast<float>(SVOVolume.NumLayers) * Config.NodeSizePreference;
					if (Config.Heuristic == ESVONavHeuristic::Euclidean) {
						Cost *= (CurrentLocation - AdjacentLocation).Size();
					}
					MinGScore = G[CurrentEdge] + Cost;
				}
				else {
					G.Add(CurrentEdge, FLT_MAX);
				}

				if (MinGScore >= (G.Contains(AdjacentEdge) ? G[AdjacentEdge] : FLT_MAX)) continue;
				Parent.Add(AdjacentEdge, CurrentEdge);
				G.Add(AdjacentEdge, MinGScore);
				// Greedy A* multiplies the heuristic score by the estimate weight
				F.Add(AdjacentEdge, G[AdjacentEdge] + Config.EstimateWeight * HeuristicScore(AdjacentEdge, InTargetLink));
			}
		}
		I++;
	}
	
#if WITH_EDITOR
	UE_LOG(LogSVONav, Display, TEXT("Pathfinding failed, iterations : %i"), I);
#endif
	return 0;
}

int SVONavPathFinder::FindPathAStar(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink,
                                    const FVector& InStartLocation, const FVector& InTargetLocation, FSVONavPathFindingConfig InConfig,
                                    FSVONavPathSharedPtr* InPath)
{
	
	OpenSet.Empty();

	ClosedSet.Empty();
	Parent.Empty();
	FScore.Empty();
	GScore.Empty();
	CurrentLink = FSVONavLink();
	TargetLink = InTargetLink;
	StartLink = InStartLink;

	OpenSet.Add(InStartLink);
	Parent.Add(InStartLink, InStartLink);
	GScore.Add(InStartLink, 0);
	FScore.Add(InStartLink, HeuristicScore(InStartLink, TargetLink)); // Distance to target

	int numIterations = 0;

	while (OpenSet.Num() > 0)
	{
		float lowestScore = FLT_MAX;
		for (FSVONavLink& link : OpenSet)
		{
			if (!FScore.Contains(link) || FScore[link] < lowestScore)
			{
				lowestScore = FScore[link];
				CurrentLink = link;
			}
		}

		OpenSet.Remove(CurrentLink);

		ClosedSet.Add(CurrentLink);

		if (CurrentLink == TargetLink)
		{
			BuildPath(Parent, CurrentLink, InStartLocation, InTargetLocation, InPath);
			return numIterations;
		}

		const FSVONavNode& currentNode = SVOVolume.GetNode(CurrentLink);

		TArray<FSVONavLink> neighbours;

		if (CurrentLink.GetLayerIndex() == 0 && currentNode.FirstChild.IsValid())
		{
			SVOVolume.GetNeighbourLeaves(CurrentLink, neighbours);
		}
		else
		{
			SVOVolume.GetNeighbourLinks(CurrentLink, neighbours);
		}

		for (const FSVONavLink& neighbour : neighbours)
		{
			ProcessLink(neighbour);
		}

		numIterations++;
	}
#if WITH_EDITOR
	UE_LOG(LogSVONav, Display, TEXT("Pathfinding failed, iterations : %i"), numIterations);
#endif
	return 0;
}

void SVONavPathFinder::ApplyPathPruning(FSVONavPathSharedPtr* Path, const FSVONavPathFindingConfig InConfig) const
{
	if (!World || InConfig.PathPruning == ESVONavPathPruning::None || Path->Get()->Points.Num() < 3) return;
	FSVONavPath PrunedPath;
	PrunedPath.Add(Path->Get()->Points[0]);
	int32 CurrentPoint = 0;
	float Radius = 0;
	if (InConfig.PathPruning == ESVONavPathPruning::WithClearance)
	{
		Radius = FMath::Max(50.f, NavComp->GetOwner()->GetComponentsBoundingBox(true).GetExtent().GetMax());
	}
	while (CurrentPoint < Path->Get()->Points.Num())
	{
		for (int32 I = CurrentPoint; I < Path->Get()->Points.Num(); I++)
		{
			if (I >= Path->Get()->Points.Num() - 2)
			{
				PrunedPath.Add(Path->Get()->Points[Path->Get()->Points.Num() - 1]);
				CurrentPoint = Path->Get()->Points.Num();
				break;
			}
			FCollisionQueryParams CollisionQueryParams;
			CollisionQueryParams.bTraceComplex = true;
			CollisionQueryParams.TraceTag = "Nav3DPathPrune";
			FHitResult HitResult;
			FVector Start = Path->Get()->Points[CurrentPoint].Location;
			FVector End = Path->Get()->Points[I + 2].Location;

			if (InConfig.PathPruning == ESVONavPathPruning::WithClearance)
			{
				World->SweepSingleByChannel(
					HitResult,
					Start,
					End,
					FQuat::Identity,
					SVOVolume.CollisionChannel,
					FCollisionShape::MakeSphere(Radius),
					CollisionQueryParams
				);
			}
			else
			{
				World->LineTraceSingleByChannel(
					HitResult,
					Start,
					End,
					SVOVolume.CollisionChannel,
					CollisionQueryParams
				);
			}

			if (HitResult.bBlockingHit)
			{
				PrunedPath.Add(Path->Get()->Points[I + 1]);
				CurrentPoint = I + 1;
				break;
			}
		}
	}
	*Path->Get() = PrunedPath;
}

void SVONavPathFinder::ApplyPathLineOfSight(FSVONavPathSharedPtr* InPath, AActor* Target, float MinimumDistance) const
{
	
}

void SVONavPathFinder::ApplyPathSmoothing(FSVONavPathSharedPtr* InPath, FSVONavPathFindingConfig Config)
{
	if (Config.PathSmoothing < 1 || InPath->Get()->GetPoints().Num() < 3) return;
	TArray<FVector> PathPoints;
	InPath->Get()->GetPath(PathPoints);

	// Duplicate the start and end points to ensure a smooth curve
	PathPoints.Insert(InPath->Get()->Points[0].Location, 0);
	PathPoints.Add(InPath->Get()->Points[InPath->Get()->Points.Num() - 1].Location);
	FSVONavPath SplinePoints;

	// Add the first path point to the spline
	SplinePoints.Add(InPath->Get()->Points[0]);
	for (int32 Index = 0; Index < PathPoints.Num(); Index++)
	{
		if (Index == 0 || Index == PathPoints.Num() - 2 || Index == PathPoints.Num() - 1) continue;
		FVector P0 = PathPoints[Index - 1];
		FVector P1 = PathPoints[Index];
		FVector P2 = PathPoints[Index + 1];
		FVector P3 = PathPoints[Index + 2];
		for (int I = 1; I <= Config.PathSmoothing; I++)
		{
			const float T = I * (1.f / Config.PathSmoothing);
			const FVector A = 2.f * P1;
			const FVector B = P2 - P0;
			const FVector C = 2.f * P0 - 5.f * P1 + 4.f * P2 - P3;
			const FVector D = -P0 + 3.f * P1 - 3.f * P2 + P3;
			SplinePoints.Add(FSVONavPathPoint(0.5f * (A + (B * T) + (C * T * T) + (D * T * T * T)),
			                                  InPath->Get()->GetPoints()[Index - 1].Layer));
		}
	}

	// Add the final path point to the spline
	SplinePoints.Add(InPath->Get()->Points[InPath->Get()->Points.Num() - 1]);
	*InPath->Get() = SplinePoints;
}


float SVONavPathFinder::HeuristicScore(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink)
{
	float Score;
	FVector StartLocation, TargetLocation;

	SVOVolume.GetLinkLocation(InStartLink, StartLocation);
	SVOVolume.GetLinkLocation(InTargetLink, TargetLocation);

	switch (Config.Heuristic)
	{
	case ESVONavHeuristic::Manhattan:
		Score = FMath::Abs(TargetLocation.X - StartLocation.X) + FMath::Abs(TargetLocation.Y - StartLocation.Y) +
			FMath::Abs(TargetLocation.Z - StartLocation.Z);
		break;
	case ESVONavHeuristic::Euclidean:
	default:
		Score = (StartLocation - TargetLocation).Size();
		break;
	}

	Score *= (1.0f - (static_cast<float>(InTargetLink.GetLayerIndex()) / static_cast<float>(SVOVolume.NumLayers)) *
		Config
		.NodeSizePreference);
	return Score;
}

float SVONavPathFinder::GetCost(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink)
{
	float Cost;

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
		Cost = (StartPos - TargetPos).Size();
	}

	Cost *= (1.0f - (static_cast<float>(InStartLink.GetLayerIndex()) / static_cast<float>(SVOVolume.NumLayers)) * Config
		.NodeSizePreference);

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

	for (int i = Points.Num() - 1; i >= 0; i--)
	{
		InPath->Get()->GetPoints().Add(Points[i]);
	}
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
			float Size = Point.Layer == 0
				             ? Volume.GetVoxelScale(Point.Layer) * 0.25f
				             : Volume.GetVoxelScale(Point.Layer) * 0.5f;

			DrawDebugBox(InWorld, Point.Location, FVector(Size), /*Volume.GetLayerColour(Point.Layer)*/ FColor::Purple,
			             true, -1.f, 0, 10.f);

			//DrawDebugSphere(InWorld, Point.Location + OffSet, 30.f, 20, FColor::Cyan, true, -1.f, 0, 20.f);

			DrawDebugLine(InWorld, Point.Location + OffSet, Points[i + 1].Location, FColor::Cyan, true, -1.f, 0, 20.f);
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

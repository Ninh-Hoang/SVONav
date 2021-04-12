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
	case ESVONavAlgorithm::HierarchicalAStar:
		Result = FindPathHierarchical(InStartLink, InTargetLink, InStartLocation, InTargetLocation, InConfig, InPath);
		break;
	case ESVONavAlgorithm::GreedyAStar:
		Result = FindPathAStar(InStartLink, InTargetLink, InStartLocation, InTargetLocation, InConfig, InPath);
		break;
	case ESVONavAlgorithm::Testing:
		Result = FindPathTesting(InStartLink, InTargetLink, InStartLocation, InTargetLocation, InConfig, InPath);
		break;
	default:
		Result = FindPathHierarchical(InStartLink, InTargetLink, InStartLocation, InTargetLocation, InConfig, InPath);
		break;
	}
#if WITH_EDITOR
	UE_LOG(LogSVONav, Display, TEXT("Pathfinding complete, iterations : %i"), Result);

	const float Duration = std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - StartTime).
		count() / 1000.0f;
	UE_LOG(LogSVONav, Display, TEXT("Path finding Time : %f"), Duration);
#endif

	if (Result != 0)
	{
		ApplyPathPruning(InPath, InConfig);
		ApplyPathSmoothing(InPath, InConfig);
	}

	return Result;
}

int SVONavPathFinder::FindPathHierarchical(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink,
                                           const FVector& InStartLocation, const FVector& InTargetLocation,
                                           FSVONavPathFindingConfig InConfig,
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

	int32 ParentLayer = 0;
	int32 ParentIndex = 0;
	FSVONavLink TopStartLink;
	FSVONavLink TopTargetLink;
	TArray<FSVONavLink> StartLinkLevels;
	TArray<FSVONavLink> TargetLinkLevels;
	if (!NavComp->DoesPathExistInternal(StartLink,
	                                    TargetLink,
	                                    ParentLayer,
	                                    ParentIndex,
	                                    TopStartLink,
	                                    TopTargetLink,
	                                    StartLinkLevels,
	                                    TargetLinkLevels))
	{
#if WITH_EDITOR
		UE_LOG(LogSVONav, Display, TEXT("Pathfinding failed, no path exist"));
#endif
		return 0;
	}
	//calculate distance between start and end
	float Distance = (InTargetLocation - InStartLocation).Size();
	int32 SearchLevel = 0;
	for(int32 I = 0; I < HieVolume.NumLayers; I++)
	{
		if(Distance < HieVolume.GetVoxelHalfSize()[I] * 2)
		{
			SearchLevel = I == 0? I : I -1;
			break;
		}
	}

	TopStartLink = StartLinkLevels[SearchLevel];
	TopTargetLink = TargetLinkLevels[SearchLevel];
	
	OpenSet.Add(TopStartLink);
	Parent.Add(TopStartLink, TopStartLink);
	GScore.Add(TopStartLink, 0);
	FScore.Add(TopStartLink, HeuristicScoreHie(TopStartLink, TopTargetLink)); // Distance to target

	int numIterations = 0;

	while (OpenSet.Num() > 0)
	{
		//get lowest score link
		float lowestScore = FLT_MAX;
		for (FSVONavLink& link : OpenSet)
		{
			if (!FScore.Contains(link) || FScore[link] < lowestScore)
			{
				lowestScore = FScore[link];
				CurrentLink = link;
			}
		}

		//explored link, remove from openset and added to closedset
		OpenSet.Remove(CurrentLink);
		ClosedSet.Add(CurrentLink);

		// confirm path found
		if (CurrentLink == TopTargetLink)
		{
			BuildHierarchicalPath(Parent,
			                      TopStartLink,
			                      TopTargetLink,
			                      StartLink,
			                      TargetLink,
			                      CurrentLink,
			                      InStartLocation,
			                      InTargetLocation,
			                      InPath);
			//BuildPathHie(Parent, CurrentLink, InStartLocation, InTargetLocation, InPath);
			return numIterations;
		}

		//get all neighbour links of node that current link point to
		TArray<FSVONavLink> neighbours;
		HieVolume.GetNeighbourLinks(CurrentLink, neighbours);

		//check each links if they has been explored, score and add to set
		for (const FSVONavLink& neighbour : neighbours)
		{
			ProcessLinkHie(neighbour);
		}

		numIterations++;
	}
#if WITH_EDITOR
	UE_LOG(LogSVONav, Display, TEXT("Pathfinding failed, iterations : %i"), numIterations);
#endif
	return 0;
}

void SVONavPathFinder::BuildHierarchicalPath(TMap<FSVONavLink, FSVONavLink>& InParent,
                                             FSVONavLink TopStartLink,
                                             FSVONavLink TopTargetLink,
                                             FSVONavLink InStartLink,
                                             FSVONavLink InTargetLink,
                                             FSVONavLink InCurrentLink,
                                             const FVector& InStartLocation,
                                             const FVector& InTargetLocation,
                                             FSVONavPathSharedPtr* InPath)
{
	FSVONavPathPoint Location;

	TArray<FSVONavPathPoint> Points;

	if (!InPath || !InPath->IsValid())
		return;
	
	/*HieVolume.GetLinkLocation(InCurrentLink, Location.Location);
	Points.Insert(Location, 0);
	Points[0].Layer = InCurrentLink.GetLayerIndex();
	Points[0].Index = InCurrentLink.GetNodeIndex();*/
	
	while (InParent.Contains(InCurrentLink) && !(InCurrentLink == InParent[InCurrentLink]))
	{
		//InCurrentLink = InParent[InCurrentLink];
		HieVolume.GetLinkLocation(InCurrentLink, Location.Location);
		Points.Insert(Location, 0);

		Points[0].Layer = InCurrentLink.GetLayerIndex();
		Points[0].Index = InCurrentLink.GetNodeIndex();
		InCurrentLink = InParent[InCurrentLink];
	}
	/*if (Points.Num() > 1)
	{
		//first point always contain lowest level link info
		Points[0].Location = InStartLocation;
		Points[0].Layer = InStartLink.LayerIndex;
		Points[0].Index = InStartLink.NodeIndex;
		Points[Points.Num() - 1].Location = InTargetLocation;
		Points[Points.Num() - 1].Layer = InTargetLink.LayerIndex;
		Points[Points.Num() - 1].Index = InTargetLink.NodeIndex;
	}
	else if (Points.Num() == 1)*/
	if (Points.Num() > 0)
	{
		FSVONavPathPoint StartPoint = FSVONavPathPoint(InStartLocation,
                       InStartLink.GetLayerIndex(),
                       InStartLink.GetNodeIndex(),
                       false);
		Points.Insert(StartPoint, 0);
		Points.Emplace(InTargetLocation,
                               InTargetLink.GetLayerIndex(),
                               InTargetLink.GetNodeIndex(),
                               false);
	}
		// If start and end are in the same voxel, just use the start and target positions.
	else if (Points.Num() == 0)
	{
		Points.Emplace(InStartLocation,
		               InStartLink.GetLayerIndex(),
		               InStartLink.GetNodeIndex(),
		               false);
		Points.Emplace(InTargetLocation,
		               InTargetLink.GetLayerIndex(),
		               InTargetLink.GetNodeIndex(),
		               false);
	}

	UE_LOG(LogSVONav, Display, TEXT("Top Hierarchical Path Num: %i"), Points.Num());
	for (int i = 0; i < Points.Num(); i++)
	{
		Points[i].Refined = HieVolume.GetNode(Points[i]).HasChildren() ? false : true;
		InPath->Get()->GetPoints().Add(Points[i]);
		UE_LOG(LogSVONav, Display, TEXT("Point - Layer: %i, Index: %i, Refined: %s"), Points[i].Layer, Points[i].Index,
		       Points[i].Refined? TEXT("true") : TEXT("false"));
	}

	TArray<FSVONavPathPoint> PathPoints = InPath->Get()->GetPoints();
	RefineHierarchicalPath(PathPoints[0].GetLink(), PathPoints[1].GetLink(), InPath, 0);

	PathPoints = InPath->Get()->GetPoints();
	//RefineHierarchicalPath(PathPoints[0].GetLink(), PathPoints[1].GetLink(), InPath, 0);
	/*bool Refined = false;s
	while (!Refined)
	{
		bool FoundUnrefinedPoint = false;
		TArray<FSVONavPathPoint> PathPoints = InPath->Get()->GetPoints();
		for(int32 I = 1; I < PathPoints.Num(); I ++)
		{
			if(!PathPoints[I].Refined)
			{
				FoundUnrefinedPoint = true;
				RefineHierarchicalPath(PathPoints[I-1].GetLink(), PathPoints[I].GetLink(), InPath, I-1);
				break;
			}
		}
s
		if(FoundUnrefinedPoint) continue;
		Refined = true;
	}*/
	/*TArray<FSVONavPathPoint> PostPath = InPath->Get()->GetPoints();
	RefineHierarchicalPath(PostPath[PostPath.Num()-2].GetLink(), PostPath[PostPath.Num()-1].GetLink(), InPath, PostPath.Num()-2);

	Refined = false;
	while (!Refined)
	{
		bool FoundUnrefinedPoint = false;
		TArray<FSVONavPathPoint> PathPoints = InPath->Get()->GetPoints();
		for(int32 I = 1; I < PathPoints.Num(); I ++)
		{
			if(!PathPoints[I].Refined)
			{
				FoundUnrefinedPoint = true;
				RefineHierarchicalPath(PathPoints[I-1].GetLink(), PathPoints[I].GetLink(), InPath, I-1);
				break;
			}
		}

		if(FoundUnrefinedPoint) continue;
		Refined = true;
	}*/
}

int SVONavPathFinder::RefineHierarchicalPath(FSVONavLink InStartLink, FSVONavLink InTargetLink,
                                             FSVONavPathSharedPtr* InPath, int32 RefineIndex)
{
	TargetSet.Empty();
	OpenSet.Empty();
	ClosedSet.Empty();
	Parent.Empty();
	FScore.Empty();
	GScore.Empty();
	CurrentLink = FSVONavLink();
	StartLink = InStartLink;
	TargetLink = InTargetLink;
	const FSVONavNode& TargetNode = HieVolume.GetNode(TargetLink);

	if (TargetNode.HasChildren()) for (auto& Child : TargetNode.Children)TargetSet.Add(Child);
	else TargetSet.Add(InTargetLink);

	OpenSet.Add(StartLink);
	Parent.Add(StartLink, StartLink);
	GScore.Add(StartLink, 0);
	FScore.Add(StartLink, HeuristicScoreHie(StartLink, TargetLink)); // Distance to target

	int numIterations = 0;

	while (OpenSet.Num() > 0)
	{
		//get lowest score link
		float lowestScore = FLT_MAX;
		for (FSVONavLink& link : OpenSet)
		{
			if (!FScore.Contains(link) || FScore[link] < lowestScore)
			{
				lowestScore = FScore[link];
				CurrentLink = link;
			}
		}

		//explored link, remove from openset and added to closedset
		OpenSet.Remove(CurrentLink);
		ClosedSet.Add(CurrentLink);

		// confirm path found
		if (TargetSet.Contains(CurrentLink))
		{
			FVector TargetLocation;
			HieVolume.GetLinkLocation(CurrentLink, TargetLocation);
			RollHierarchicalPath(Parent,
			                     CurrentLink,
			                     StartLink,
			                     CurrentLink,
			                     InPath->Get()->GetPoints()[RefineIndex].Location,
			                     TargetLocation,
			                     InPath,
			                     RefineIndex);
#if WITH_EDITOR
			UE_LOG(LogSVONav, Display, TEXT("Path Rolling success, number of Iteration: %i"), numIterations);
#endif
			return 1;
		}

		//get all neighbour links of node that current link point to
		TArray<FSVONavLink> neighbours;
		HieVolume.GetNeighbourLinks(CurrentLink, neighbours);

		//check each links if they has been explored, score and add to set
		for (const FSVONavLink& neighbour : neighbours)
		{
			ProcessLinkHie(neighbour);
		}

		numIterations++;
	}
#if WITH_EDITOR
	UE_LOG(LogSVONav, Display, TEXT("Path Rolling fail, can't find refined path: number of Iteration: %i"),
	       numIterations);
#endif
	return 0;
}

void SVONavPathFinder::RollHierarchicalPath(TMap<FSVONavLink, FSVONavLink>& InParent,
                                            FSVONavLink InCurrentLink,
                                            FSVONavLink InStartLink,
                                            FSVONavLink InTargetLink,
                                            const FVector& InStartLocation,
                                            const FVector& InTargetLocation,
                                            FSVONavPathSharedPtr* InPath,
                                            int32& PointIndex)
{
	FSVONavPathPoint Location;

	TArray<FSVONavPathPoint> Points;

	if (!InPath || !InPath->IsValid())
		return;

	while (InParent.Contains(InCurrentLink) && !(InCurrentLink == InParent[InCurrentLink]))
	{
		InCurrentLink = InParent[InCurrentLink];
		HieVolume.GetLinkLocation(InCurrentLink, Location.Location);
		Points.Insert(Location, 0);

		Points[0].Layer = InCurrentLink.GetLayerIndex();
		Points[0].Index = InCurrentLink.GetNodeIndex();
		Points[0].Refined = HieVolume.GetNode(InCurrentLink).HasChildren() ? false : true;
	}

	TArray<FSVONavPathPoint> PathPoints = InPath->Get()->GetPoints();

	// If start and end are in the same voxel, just use the start and target positions.
	if (Points.Num() > 1)
	{
		//first point always contain lowest level link info
		Points[0].Location = InStartLocation;
		Points[0].Layer = InStartLink.LayerIndex;
		Points[0].Index = InStartLink.NodeIndex;

		int32 Index = Points.Emplace();
		Points[Index].Location = InTargetLocation;
		Points[Index].Layer = InTargetLink.LayerIndex;
		Points[Index].Index = InTargetLink.NodeIndex,
			Points[Index].Refined = HieVolume.GetNode(InTargetLink).HasChildren() ? false : true;
	}
	else if (Points.Num() == 0)
	{
		Points.Emplace(InStartLocation,
		               InStartLink.GetLayerIndex(),
		               InStartLink.GetNodeIndex(),
		               HieVolume.GetNode(InStartLink).HasChildren() ? false : true);

		Points.Emplace(InTargetLocation,
		               InTargetLink.GetLayerIndex(),
		               InTargetLink.GetNodeIndex(),
		               HieVolume.GetNode(InTargetLink).HasChildren() ? false : true);
	}
	PathPoints.Insert(Points, PointIndex + 1);
	PathPoints.RemoveAt(PointIndex + Points.Num() + 1);
	PathPoints.RemoveAt(PointIndex);

	InPath->Get()->Empty();
	for (int i = 0; i < PathPoints.Num(); i++)
	{
		InPath->Get()->GetPoints().Add(PathPoints[i]);
	}

#if WITH_EDITOR
	UE_LOG(LogSVONav, Display, TEXT("Path Rolling finish, Num of total Point: %i, Point added %i"), PathPoints.Num(),
	       Points.Num()-2);
#endif
}

int SVONavPathFinder::FindPathTesting(const FSVONavLink& InStartLink,
                                      const FSVONavLink& InTargetLink,
                                      const FVector& InStartLocation,
                                      const FVector& InTargetLocation,
                                      FSVONavPathFindingConfig InConfig,
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
	while (OpenSet.Num() > 0)
	{
		float LowestScore = FLT_MAX;
		for (FSVONavLink& Edge : OpenSet)
		{
			if (!F.Contains(Edge) || F[Edge] < LowestScore)
			{
				LowestScore = F[Edge];
				CurrentEdge = Edge;
			}
		}
		OpenSet.Remove(CurrentEdge);
		ClosedSet.Add(CurrentEdge);

		if (CurrentEdge.NodeIndex == InTargetLink.NodeIndex)
		{
			FSVONavPathPoint PathPoint;

			while (Parent.Contains(CurrentEdge) && !(CurrentEdge == Parent[CurrentEdge]))
			{
				CurrentEdge = Parent[CurrentEdge];
				SVOVolume.GetLinkLocation(CurrentEdge, PathPoint.Location);
				InPath->Get()->Points.Insert(PathPoint, 0);
				const FSVONavNode& Node = SVOVolume.GetNode(CurrentEdge);
				if (CurrentEdge.GetLayerIndex() == 0)
				{
					if (!Node.HasChildren()) InPath->Get()->Points[0].Layer = 1;
					else InPath->Get()->Points[0].Layer = 0;
				}
				else
				{
					InPath->Get()->Points[0].Layer = CurrentEdge.GetLayerIndex() + 1;
				}
			}

			if (InPath->Get()->Points.Num() > 1)
			{
				InPath->Get()->Points[0].Location = InStartLocation;
				InPath->Get()->Points[InPath->Get()->Points.Num() - 1].Location = InTargetLocation;
			}
			else
			{
				if (InPath->Get()->Points.Num() == 0) InPath->Get()->Points.Emplace();
				InPath->Get()->Points[0].Location = InTargetLocation;
				InPath->Get()->Points.Emplace(InStartLocation,
				                              InStartLink.GetLayerIndex(),
				                              InStartLink.GetNodeIndex(),
				                              HieVolume.GetNode(InStartLink).HasChildren() ? false : true);
			}

			return I;
		}

		const FSVONavNode& CurrentNode = SVOVolume.GetNode(CurrentEdge);
		TArray<FSVONavLink> AdjacentEdges;

		if (CurrentEdge.GetLayerIndex() == 0 && CurrentNode.FirstChild.IsValid())
		{
			SVOVolume.GetNeighbourLeaves(CurrentEdge, AdjacentEdges);
		}
		else
		{
			SVOVolume.GetNeighbourLinks(CurrentEdge, AdjacentEdges);
		}

		for (auto& AdjacentEdge : AdjacentEdges)
		{
			if (AdjacentEdge.IsValid())
			{
				if (ClosedSet.Contains(AdjacentEdge))
				{
					continue;
				}
				if (!OpenSet.Contains(AdjacentEdge))
				{
					OpenSet.Add(AdjacentEdge);
				}

				float MinGScore = FLT_MAX;
				if (G.Contains(CurrentEdge))
				{
					FVector CurrentLocation(0.f), AdjacentLocation(0.f);
					SVOVolume.GetLinkLocation(CurrentEdge, CurrentLocation);
					SVOVolume.GetLinkLocation(AdjacentEdge, AdjacentLocation);
					float Cost = 0.0f;

					Cost -= static_cast<float>(InTargetLink.GetLayerIndex()) / static_cast<float>(SVOVolume.NumLayers) *
						Config.NodeSizePreference;
					if (Config.Heuristic == ESVONavHeuristic::Euclidean)
					{
						Cost *= (CurrentLocation - AdjacentLocation).Size();
					}
					MinGScore = G[CurrentEdge] + Cost;
				}
				else
				{
					G.Add(CurrentEdge, FLT_MAX);
				}

				if (MinGScore >= (G.Contains(AdjacentEdge) ? G[AdjacentEdge] : FLT_MAX)) continue;
				Parent.Add(AdjacentEdge, CurrentEdge);
				G.Add(AdjacentEdge, MinGScore);
				// Greedy A* multiplies the heuristic score by the estimate weight
				F.Add(AdjacentEdge,
				      G[AdjacentEdge] + Config.EstimateWeight * HeuristicScore(AdjacentEdge, InTargetLink));
			}
		}
		I++;
	}

#if WITH_EDITOR
	UE_LOG(LogSVONav, Display, TEXT("Pathfinding failed, iterations : %i"), I);
#endif
	return 0;
}

int SVONavPathFinder::FindPathAStar(const FSVONavLink& InStartLink,
                                    const FSVONavLink& InTargetLink,
                                    const FVector& InStartLocation,
                                    const FVector& InTargetLocation,
                                    FSVONavPathFindingConfig InConfig,
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
			CollisionQueryParams.TraceTag = "SVONavPathPrune";
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
					HieVolume.CollisionChannel,
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
					HieVolume.CollisionChannel,
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
			                                  InPath->Get()->GetPoints()[Index - 1].Layer,
			                                  InPath->Get()->GetPoints()[Index - 1].Index, true));
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

float SVONavPathFinder::HeuristicScoreHie(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink)
{
	float Score;
	FVector StartLocation, TargetLocation;

	HieVolume.GetLinkLocation(InStartLink, StartLocation);
	HieVolume.GetLinkLocation(InTargetLink, TargetLocation);

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

	Score *= (1.0f - (static_cast<float>(InTargetLink.GetLayerIndex()) / static_cast<float>(HieVolume.NumLayers)) *
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

float SVONavPathFinder::GetCostHie(const FSVONavLink& InStartLink, const FSVONavLink& InTargetLink)
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
		HieVolume.GetLinkLocation(InStartLink, StartPos);
		HieVolume.GetLinkLocation(InTargetLink, TargetPos);
		Cost = (StartPos - TargetPos).Size();
	}

	Cost *= (1.0f - (static_cast<float>(InStartLink.GetLayerIndex()) / static_cast<float>(HieVolume.NumLayers)) * Config
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

void SVONavPathFinder::ProcessLinkHie(const FSVONavLink& NeighbourLink)
{
	if (NeighbourLink.IsValid())
	{
		//already explored
		if (ClosedSet.Contains(NeighbourLink))
			return;

		//not yet added to pending set
		if (!OpenSet.Contains(NeighbourLink))
		{
			OpenSet.Add(NeighbourLink);
		}

		float MaxGScore = FLT_MAX;
		if (GScore.Contains(CurrentLink))
			MaxGScore = GScore[CurrentLink] + GetCostHie(CurrentLink, NeighbourLink);
		else
			GScore.Add(CurrentLink, FLT_MAX);

		if (MaxGScore >= (GScore.Contains(NeighbourLink) ? GScore[NeighbourLink] : FLT_MAX))
			return;

		Parent.Add(NeighbourLink, CurrentLink);
		GScore.Add(NeighbourLink, MaxGScore);
		FScore.Add(NeighbourLink,
		           GScore[NeighbourLink] + (Config.EstimateWeight * HeuristicScoreHie(NeighbourLink, TargetLink)));
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
		Points.Emplace(InStartLocation,
		               StartLink.GetLayerIndex(),
		               StartLink.GetNodeIndex(),
		               HieVolume.GetNode(StartLink).HasChildren() ? false : true);
	}

	for (int i = Points.Num() - 1; i >= 0; i--)
	{
		InPath->Get()->GetPoints().Add(Points[i]);
	}
}

void SVONavPathFinder::BuildPathHie(TMap<FSVONavLink, FSVONavLink>& InParent, FSVONavLink InCurrentLink,
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
		HieVolume.GetLinkLocation(InCurrentLink, Location.Location);
		Points.Add(Location);
		const FSVONavNode& node = HieVolume.GetNode(InCurrentLink);
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
		Points.Emplace(InStartLocation,
		               StartLink.GetLayerIndex(),
		               StartLink.GetNodeIndex(),
		               HieVolume.GetNode(StartLink).HasChildren() ? false : true);
	}

	for (int i = Points.Num() - 1; i >= 0; i--)
	{
		InPath->Get()->GetPoints().Add(Points[i]);
		UE_LOG(LogSVONav, Display, TEXT("Point Layer : %i"), Points[i].Layer);
	}
}

#if WITH_EDITOR

void SVONavPathFinder::DrawDebug(UWorld* InWorld, const ASVONavVolumeBase& Volume, FSVONavPathSharedPtr* InPath) const
{
	TArray<FSVONavPathPoint> Points = InPath->Get()->GetPoints();
	for (int i = 0; i < Points.Num(); i++)
	{
		FSVONavPathPoint& Point = Points[i];

		FVector OffSet(0.f);
		//if (i == 0)
		//offSet.Z -= 300.f;
		float Size = Point.Layer == 0
                         ? HieVolume.GetVoxelScale(Point.Layer) * 0.5f
                         : HieVolume.GetVoxelScale(Point.Layer) * 0.5f;
		if (i < Points.Num() - 1)
		{

			DrawDebugBox(InWorld, Point.Location, FVector(Size), /*Volume.GetLayerColour(Point.Layer)*/ FColor::Purple,
			             true, -1.f, 0, 10.f);
			DrawDebugLine(InWorld, Point.Location + OffSet, Points[i + 1].Location, FColor::Cyan, true, -1.f, 0, 20.f);
		}
		else
		{
			DrawDebugBox(InWorld, Point.Location, FVector(Size), /*Volume.GetLayerColour(Point.Layer)*/ FColor::Purple,
                         true, -1.f, 0, 10.f);
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

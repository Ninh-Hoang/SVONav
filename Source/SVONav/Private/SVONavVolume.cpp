// Fill out your copyright notice in the Description page of Project Settings.

#include "SVONavVolume.h"
#include "Components/BrushComponent.h"
#include "Components/LineBatchComponent.h"
#include "Engine/CollisionProfile.h"
#include "GameFramework/PlayerController.h"
#include "Engine/Public/DrawDebugHelpers.h"
#include "chrono"
#include "Editor.h"
#include "Builders/CubeBuilder.h"
#include "Async/ParallelFor.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"

#include "chrono"
#include "SVONavUpdateOctreeTask.h"
#include "Async/Async.h"
using namespace std::chrono;

ASVONavVolume::ASVONavVolume(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer),
	  VolumeOrigin(FVector::ZeroVector),
	  VolumeExtent(FVector::ZeroVector)
{
	GetBrushComponent()->Mobility = EComponentMobility::Movable;
	PrimaryActorTick.bCanEverTick = true;

	RegisterAllActorTickFunctions(true, false);
	CollisionQueryParams.bFindInitialOverlaps = true;
	CollisionQueryParams.bTraceComplex = false;
	CollisionQueryParams.TraceTag = "SVONavRasterize";
}

void ASVONavVolume::UpdateTaskComplete()
{
	UnlockOctree();

#if WITH_EDITOR
	// Run the debug draw on the game thread
	AsyncTask(ENamedThreads::GameThread, [=]()
	{
		DebugDrawOctree();
		DebugDrawOctree_Hie();
	});
#endif
}

void ASVONavVolume::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
}

void ASVONavVolume::BeginPlay()
{
	CachedOctree = Octree;
	OnUpdateComplete.BindUObject(this, &ASVONavVolume::UpdateTaskComplete);
	SetActorTickInterval(TickInterval);
}

void ASVONavVolume::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!bOctreeLocked)
	{
		if (bUpdateRequested)
		{
			// Prevent any further requests until update task is complete
			LockOctree();

			// Execute UpdateOctree as background task
			//(new FAutoDeleteAsyncTask<FSVONavUpdateOctreeTask>(this, OnUpdateComplete))->StartBackgroundTask();

			// Update complete
			bUpdateRequested = false;

#if WITH_EDITOR
		}
		else if (bDebugDrawRequested)
		{
			DebugDrawOctree();
			DebugDrawOctree_Hie();
			bDebugDrawRequested = false;
#endif
		}
	}
}

void ASVONavVolume::PostRegisterAllComponents()
{
	Super::PostRegisterAllComponents();
}

void ASVONavVolume::PostUnregisterAllComponents()
{
	Super::PostUnregisterAllComponents();
}

void ASVONavVolume::UpdateOctree()
{
#if WITH_EDITOR
	const auto UpdateStartTime = high_resolution_clock::now();
#endif

	Octree = CachedOctree;
}

void ASVONavVolume::Serialize(FArchive& Ar)
{
	Super::Serialize(Ar);
	Ar << Octree;
	Ar << VoxelHalfSizes;
	//Ar << HieOctree;
	//Ar << VoxelHalfSizes_Hie;
	Ar << VolumeExtent;
	NumBytes = Octree.GetSize();
}

void ASVONavVolume::GetVolumeExtents(const FVector& Location, int32 LayerIndex, FIntVector& Extents) const
{
	const FBox ComponentBox = GetComponentsBoundingBox(true);
	FVector ComponentOrigin;
	FVector ComponentExtent;
	ComponentBox.GetCenterAndExtents(ComponentOrigin, ComponentExtent);
	const FVector LocationLocal = Location - ComponentOrigin - ComponentExtent;
	const float Scale = VoxelHalfSizes[LayerIndex];
	Extents.X = FMath::FloorToInt(LocationLocal.X / Scale);
	Extents.Y = FMath::FloorToInt(LocationLocal.Y / Scale);
	Extents.Z = FMath::FloorToInt(LocationLocal.Z / Scale);
}

void ASVONavVolume::GetMortonVoxel(const FVector& Location, int32 LayerIndex, FIntVector& MortonLocation) const
{
	const FVector LocationLocal = Location - (VolumeOrigin - VolumeExtent);
	const float Size = VoxelHalfSizes[LayerIndex] * 2;
	MortonLocation.X = FMath::FloorToInt(LocationLocal.X / Size);
	MortonLocation.Y = FMath::FloorToInt(LocationLocal.Y / Size);
	MortonLocation.Z = FMath::FloorToInt(LocationLocal.Z / Size);
}

void ASVONavVolume::UpdateVolume()
{
	// Calculate the nearest integer exponent to fit the voxel size perfectly within the volume extents
	VoxelExponent = FMath::RoundToInt(FMath::Log2(VolumeSize / (VoxelSize * 4)));
	VoxelExponent_Hie = FMath::RoundToInt(FMath::Log2(VolumeSize / (VoxelSize)));

	NumLayers = VoxelExponent + 1;
	NumLayer_Hie = VoxelExponent_Hie + 1;

	// Build a list of voxel half-scale sizes for each layer
	VoxelHalfSizes.Reset();
	VoxelHalfSizes.Reserve(NumLayers);
	for (int32 I = 0; I < NumLayers; I++) VoxelHalfSizes.Add(GetVoxelScale(I) * 0.5f);

	VoxelHalfSizes_Hie.Reset();
	VoxelHalfSizes_Hie.Reserve(NumLayer_Hie);
	for (int32 I = 0; I < NumLayer_Hie; I++) VoxelHalfSizes_Hie.Add(GetVoxelScale_Hie(I) * 0.5f);

	ActualVolumeSize = GetActualVolumeSize();
	UCubeBuilder* CubeBuilder = Cast<UCubeBuilder>(GEditor->FindBrushBuilder(UCubeBuilder::StaticClass()));
	CubeBuilder->X = ActualVolumeSize;
	CubeBuilder->Y = ActualVolumeSize;
	CubeBuilder->Z = ActualVolumeSize;
	CubeBuilder->Build(GetWorld(), this);

	const FBox Bounds = GetComponentsBoundingBox(true);
	Bounds.GetCenterAndExtents(VolumeOrigin, VolumeExtent);
}

bool ASVONavVolume::IsAnyMemberBlocked(layerindex_t LayerIndex, mortoncode_t aCode) const
{
	const mortoncode_t parentCode = aCode >> 3;

	if (LayerIndex == BlockedIndices.Num())
	{
		return true;
	}
	// The parent of this code is blocked
	if (BlockedIndices[LayerIndex].Contains(parentCode))
	{
		return true;
	}

	return false;
}

bool ASVONavVolume::GetIndexForCode(layerindex_t LayerIndex, mortoncode_t aCode, nodeindex_t& oIndex) const
{
	const TArray<FSVONavNode>& layer = GetLayer(LayerIndex);

	for (int i = 0; i < layer.Num(); i++)
	{
		if (layer[i].MortonCode == aCode)
		{
			oIndex = i;
			return true;
		}
	}

	return false;
}

bool ASVONavVolume::FindLink(uint8 LayerIndex, int32 NodeIndex, uint8 Direction, FSVONavLink& Link,
                             const FVector& NodeLocation)
{
	int32 maxCoord = GetSegmentNodeCount(LayerIndex);
	FSVONavNode& node = GetLayer(LayerIndex)[NodeIndex];
	TArray<FSVONavNode>& layer = GetLayer(LayerIndex);

	// Get our world co-ordinate
	uint_fast32_t x = 0, y = 0, z = 0;
	morton3D_64_decode(node.MortonCode, x, y, z);
	int32 sX = x, sY = y, sZ = z;
	// Add the direction
	sX += Directions[Direction].X;
	sY += Directions[Direction].Y;
	sZ += Directions[Direction].Z;

	// If the coords are out of bounds, the link is invalid.
	if (sX < 0 || sX >= maxCoord || sY < 0 || sY >= maxCoord || sZ < 0 || sZ >= maxCoord)
	{
		Link.Invalidate();

		return true;
	}
	x = sX;
	y = sY;
	z = sZ;
	// Get the morton code for the direction
	mortoncode_t thisCode = morton3D_64_encode(x, y, z);
	bool isHigher = thisCode > node.MortonCode;
	int32 nodeDelta = (isHigher ? 1 : -1);

	while ((NodeIndex + nodeDelta) < layer.Num() && NodeIndex + nodeDelta >= 0)
	{
		// This is the node we're looking for
		if (layer[NodeIndex + nodeDelta].MortonCode == thisCode)
		{
			const FSVONavNode& thisNode = layer[NodeIndex + nodeDelta];
			// This is a leaf node
			if (LayerIndex == 0 && thisNode.HasChildren())
			{
				// Set invalid link if the leaf node is completely blocked, no point linking to it
				if (GetLeafNode(thisNode.FirstChild.GetNodeIndex()).IsOccluded())
				{
					Link.Invalidate();
					return true;
				}
			}
			// Otherwise, use this link
			Link.LayerIndex = LayerIndex;
			check(NodeIndex + nodeDelta < layer.Num());
			Link.NodeIndex = NodeIndex + nodeDelta;

#if WITH_EDITOR
			FVector AdjacentLocation;
			GetNodeLocation(LayerIndex, thisCode, AdjacentLocation);
			DebugLinks.Add(FSVONavDebugLink(NodeLocation, AdjacentLocation, LayerIndex));
#endif

			return true;
		}
			// If we've passed the code we're looking for, it's not on this layer
		else if ((isHigher && layer[NodeIndex + nodeDelta].MortonCode > thisCode) || (!isHigher && layer[NodeIndex +
			nodeDelta].MortonCode < thisCode))
		{
			return false;
		}

		nodeDelta += (isHigher ? 1 : -1);
	}

	// I'm not entirely sure if it's valid to reach the end? Hmmm...
	return false;
}

const FSVONavLeafNode& ASVONavVolume::GetLeafNode(nodeindex_t aIndex) const
{
	return Octree.Leaves[aIndex];
}

float ASVONavVolume::GetVoxelScale(uint8 LayerIndex) const
{
	return VolumeExtent.X / FMath::Pow(2.0f, VoxelExponent) * FMath::Pow(2.0f, LayerIndex + 1);
}

FBox ASVONavVolume::GetBoundingBox() const
{
	const FBoxSphereBounds Bounds = GetBrushComponent()->CalcBounds(ActorToWorld());
	return Bounds.GetBox();
}

bool ASVONavVolume::GetLink(const FVector& Location, FSVONavLink& Link)
{
	if (!IsWithinBounds(Location)) return false;

	int32 LayerIndex = NumLayers - 1;
	while (LayerIndex >= 0)
	{
		const TArray<FSVONavNode>& Layer = GetLayer(LayerIndex);
		FIntVector Voxel;
		GetMortonVoxel(Location, LayerIndex, Voxel);
		const uint_fast64_t MortonCode = morton3D_64_encode(Voxel.X, Voxel.Y, Voxel.Z);
		int32 NodeIndex;
		if (GetNodeIndex(LayerIndex, MortonCode, NodeIndex))
		{
			const FSVONavNode Node = Layer[NodeIndex];

			if (!Node.FirstChild.IsValid())
			{
				Link.SetLayerIndex(LayerIndex);
				Link.SetNodeIndex(NodeIndex);
				Link.SetSubNodeIndex(0);
				return true;
			}

			if (LayerIndex == 0)
			{
				const FSVONavLeafNode Leaf = Octree.Leaves[Node.FirstChild.NodeIndex];
				const float VoxelHalfSize = VoxelHalfSizes[LayerIndex];

				FVector NodeLocation;
				GetNodeLocation(LayerIndex, Node.MortonCode, NodeLocation);
				const FVector NodeOrigin = NodeLocation - FVector(VoxelHalfSize);
				const float Leavesize = VoxelHalfSize * 0.5f;
				const float LeafHalfSize = Leavesize * 0.5f;

				bool bFound = false;
				int32 LeafIndex;
				for (LeafIndex = 0; LeafIndex < 64; LeafIndex ++)
				{
					uint_fast32_t x, y, z;
					morton3D_64_decode(LeafIndex, x, y, z);
					const FVector LeafLocation = NodeOrigin + FVector(x * Leavesize, y * Leavesize, z * Leavesize) +
						FVector(LeafHalfSize);
					if (Location.X >= LeafLocation.X - LeafHalfSize && Location.X <= LeafLocation.X + LeafHalfSize &&
						Location.Y >= LeafLocation.Y - LeafHalfSize && Location.Y <= LeafLocation.Y + LeafHalfSize &&
						Location.Z >= LeafLocation.Z - LeafHalfSize && Location.Z <= LeafLocation.Z + LeafHalfSize)
					{
						bFound = true;
						break;
					}
				}

				if (!bFound) return false;
				Link.SetLayerIndex(LayerIndex);
				Link.SetNodeIndex(NodeIndex);
				if (Leaf.GetSubNode(LeafIndex)) return false;
				Link.SetSubNodeIndex(LeafIndex);
				return true;
			}
			LayerIndex = Node.FirstChild.LayerIndex;
		}
		else if (LayerIndex == 0)
		{
			break;
		}
	}
	return false;
}

bool ASVONavVolume::FindAccessibleLink(FVector& Location, FSVONavLink& Link)
{
	for (int32 I = 1; I < 4; I++)
	{
		for (int32 J = 0; J < 6; J++)
		{
			FVector OffsetLocation = Location + FVector(Directions[J] * Clearance * I);
			if (GetLink(OffsetLocation, Link))
			{
				Location = OffsetLocation;
				return true;
			}
		}
	}
	return false;
}

const FSVONavNode& ASVONavVolume::GetNode(const FSVONavLink& Link) const
{
	return Octree.Layers[Link.LayerIndex][Link.NodeIndex];
}

bool ASVONavVolume::LinkNodeIsValid(const FSVONavLink& Link) const
{
	if (static_cast<int32>(Link.LayerIndex) >= Octree.Layers.Num()) return false;
	return Link.IsValid() && static_cast<int32>(Link.NodeIndex) < Octree.Layers[Link.LayerIndex].Num();
}

bool ASVONavVolume::GetLinkLocation(const FSVONavLink& Link, FVector& Location) const
{
	const FSVONavNode& Node = GetLayer(Link.GetLayerIndex())[Link.GetNodeIndex()];
	GetNodeLocation(Link.GetLayerIndex(), Node.MortonCode, Location);
	if (Link.GetLayerIndex() == 0 && Node.FirstChild.IsValid())
	{
		uint_fast32_t X, Y, Z;
		morton3D_64_decode(Link.GetSubNodeIndex(), X, Y, Z);
		const float Scale = VoxelHalfSizes[0] * 2;
		Location += FVector(X * Scale * 0.25f, Y * Scale * 0.25f, Z * Scale * 0.25f) - FVector(Scale * 0.375);
		const FSVONavLeafNode& Leaf = Octree.Leaves[Node.FirstChild.NodeIndex];
		return !Leaf.GetSubNode(Link.GetSubNodeIndex());
	}
	return true;
}

bool ASVONavVolume::GetNodeLocation(uint8 LayerIndex, uint_fast64_t MortonCode, FVector& Location) const
{
	const float Scale = VoxelHalfSizes[LayerIndex] * 2;
	uint_fast32_t X, Y, Z;
	morton3D_64_decode(MortonCode, X, Y, Z);
	Location = VolumeOrigin - VolumeExtent + Scale * FVector(X, Y, Z) + FVector(Scale * 0.5f);
	return true;
}

bool ASVONavVolume::GetNodeLocation(const FSVONavLink& Link, FVector& Location)
{
	const FSVONavNode& Node = Octree.Layers[Link.LayerIndex][Link.NodeIndex];
	GetNodeLocation(Link.LayerIndex, Node.MortonCode, Location);
	if (Link.LayerIndex == 0 && Node.FirstChild.IsValid())
	{
		const float Size = VoxelHalfSizes[0] * 2;
		uint_fast32_t X, Y, Z;
		morton3D_64_decode(Link.SubNodeIndex, X, Y, Z);
		Location += FVector(X * Size / 4, Y * Size / 4, Z * Size / 4) - FVector(Size * 0.375f);
		const FSVONavLeafNode& Leaf = Octree.Leaves[Node.FirstChild.NodeIndex];
		return !Leaf.GetSubNode(Link.SubNodeIndex);
	}
	return true;
}

void ASVONavVolume::GetNeighbourLeaves(const FSVONavLink& Link, TArray<FSVONavLink>& NeighbourLinks) const
{
	const uint_fast64_t LeafIndex = Link.SubNodeIndex;
	if (LinkNodeIsValid(Link))
	{
		const FSVONavNode& Node = GetNode(Link);
		if (static_cast<int32>(Node.FirstChild.NodeIndex) >= Octree.Leaves.Num()) return;
		const FSVONavLeafNode& FirstLeaf = Octree.Leaves[Node.FirstChild.NodeIndex];

		uint_fast32_t X = 0, Y = 0, Z = 0;
		morton3D_64_decode(LeafIndex, X, Y, Z);
		for (int32 I = 0; I < 6; I++)
		{
			int32 SignedX = X + Directions[I].X;
			int32 SignedY = Y + Directions[I].Y;
			int32 SignedZ = Z + Directions[I].Z;
			if (SignedX >= 0 && SignedX < 4
				&& SignedY >= 0 && SignedY < 4
				&& SignedZ >= 0 && SignedZ < 4)
			{
				uint_fast64_t Index = morton3D_64_encode(SignedX, SignedY, SignedZ);

				if (FirstLeaf.GetSubNode(Index)) continue;
				NeighbourLinks.Emplace(0, Link.NodeIndex, Index);
				continue;
			}
			const FSVONavLink& AdjacentLink = Node.Neighbours[I];

			if (LinkNodeIsValid(AdjacentLink))
			{
				const FSVONavNode& AdjacentNode = GetNode(AdjacentLink);
				if (!AdjacentNode.FirstChild.IsValid())
				{
					NeighbourLinks.Add(AdjacentLink);
					continue;
				}

				const FSVONavLeafNode& Leaf = Octree.Leaves[AdjacentNode.FirstChild.NodeIndex];

				if (!Leaf.IsOccluded())
				{
					if (SignedX < 0) SignedX = 3;
					else if (SignedX > 3) SignedX = 0;
					else if (SignedY < 0) SignedY = 3;
					else if (SignedY > 3) SignedY = 0;
					else if (SignedZ < 0) SignedZ = 3;
					else if (SignedZ > 3) SignedZ = 0;
					const uint_fast64_t SubCode = morton3D_64_encode(SignedX, SignedY, SignedZ);
					if (!Leaf.GetSubNode(SubCode))
						NeighbourLinks.
							Emplace(0, AdjacentNode.FirstChild.NodeIndex, SubCode);
				}
			}
		}
	}
}

void ASVONavVolume::GetNeighbourLinks(const FSVONavLink& Link, TArray<FSVONavLink>& NeighbourLinks) const
{
	if (!LinkNodeIsValid(Link)) return;
	const FSVONavNode& Node = GetNode(Link);
	for (int32 I = 0; I < 6; I++)
	{
		const FSVONavLink& AdjacentLink = Node.Neighbours[I];
		if (!AdjacentLink.IsValid()) continue;
		if (!LinkNodeIsValid(AdjacentLink)) continue;
		const FSVONavNode& AdjacentNode = GetNode(AdjacentLink);
		if (!AdjacentNode.HasChildren())
		{
			NeighbourLinks.Add(AdjacentLink);
			continue;
		}

		TArray<FSVONavLink> Links;
		Links.Push(AdjacentLink);

		while (Links.Num() > 0)
		{
			FSVONavLink CurrentLink = Links.Pop();
			const FSVONavNode& CurrentNode = GetNode(CurrentLink);
			if (!CurrentNode.HasChildren())
			{
				NeighbourLinks.Add(CurrentLink);
				continue;
			}
			if (CurrentLink.GetLayerIndex() > 0)
			{
				for (const int32& ChildIndex : NodeOffsets[I])
				{
					FSVONavLink LinkChild = CurrentNode.FirstChild;
					LinkChild.NodeIndex += ChildIndex;
					const FSVONavNode& NodeChild = GetNode(LinkChild);
					if (NodeChild.HasChildren()) Links.Emplace(LinkChild);
					else NeighbourLinks.Emplace(LinkChild);
				}
			}
			else
			{
				for (const int32& LeafIndex : LeafOffsets[I])
				{
					FSVONavLink LinkChild = AdjacentNode.FirstChild;
					const FSVONavLeafNode& Leaf = Octree.Leaves[LinkChild.NodeIndex];
					LinkChild.SubNodeIndex = LeafIndex;
					if (!Leaf.GetSubNode(LeafIndex))
					{
						NeighbourLinks.Emplace(LinkChild);
					}
				}
			}
		}
	}
}

int32 ASVONavVolume::GetLayerNodeCount(uint8 LayerIndex) const
{
	return FMath::Pow(8, VoxelExponent - LayerIndex);
}

int32 ASVONavVolume::GetSegmentNodeCount(uint8 LayerIndex) const
{
	return FMath::Pow(2, VoxelExponent - LayerIndex);
}

bool ASVONavVolume::GetNodeIndex(const uint8 LayerIndex, const uint_fast64_t NodeMortonCode, int32& NodeIndex) const
{
	const auto& OctreeLayer = Octree.Layers[LayerIndex];
	int32 Start = 0;
	int32 End = OctreeLayer.Num() - 1;
	int32 Mean = (Start + End) * 0.5f;

	// Binary search by Morton code
	while (Start <= End)
	{
		if (OctreeLayer[Mean].MortonCode < NodeMortonCode) Start = Mean + 1;
		else if (OctreeLayer[Mean].MortonCode == NodeMortonCode)
		{
			NodeIndex = Mean;
			return true;
		}
		else End = Mean - 1;
		Mean = (Start + End) * 0.5f;
	}
	return false;
}

int32 ASVONavVolume::GetInsertIndex(const uint8 LayerIndex, const uint_fast64_t MortonCode) const
{
	const auto& OctreeLayer = Octree.Layers[LayerIndex];
	int32 Start = 0;
	int32 End = OctreeLayer.Num() - 1;
	int32 Mean = (Start + End) * 0.5f;

	// Binary search by Morton code
	while (Start <= End)
	{
		if (OctreeLayer[Mean].MortonCode < MortonCode) Start = Mean + 1;
		else End = Mean - 1;
		Mean = (Start + End) * 0.5f;
	}
	return Start;
}

bool ASVONavVolume::IsBlocked(const FVector& Location, float Size) const
{
	return GetWorld()->OverlapBlockingTestByChannel(
		Location,
		FQuat::Identity,
		CollisionChannel,
		FCollisionShape::MakeBox(FVector(Size + Clearance)),
		CollisionQueryParams
	);
}

bool ASVONavVolume::IsBlocked(const FVector& Location, float Size, TArray<FOverlapResult>& OverlapResults) const
{
	return GetWorld()->OverlapMultiByChannel(
		OverlapResults,
		Location,
		FQuat::Identity,
		CollisionChannel,
		FCollisionShape::MakeBox(FVector(Size + Clearance)),
		CollisionQueryParams
	);
}

#if WITH_EDITOR

bool ASVONavVolume::InDebugRange(FVector Location) const
{
	if (!GetWorld()) return true;
	if (GetWorld()->ViewLocationsRenderedLastFrame.Num() == 0) return true;
	return FVector::Dist(GetWorld()->ViewLocationsRenderedLastFrame[0], Location) < DebugDistance;
}

FColor ASVONavVolume::GetLayerColour(const int32 LayerIndex) const
{
	const float Ratio = LayerColours.Num() / static_cast<float>(NumLayers) * LayerIndex;
	const int32 FirstIndex = FMath::FloorToInt(Ratio);
	const int32 LastIndex = FMath::Min(FMath::CeilToInt(Ratio), LayerColours.Num() - 1);
	const float Lerp = FMath::Fmod(Ratio, 1);
	return FColor(
        FMath::Lerp(LayerColours[FirstIndex].R, LayerColours[LastIndex].R, Lerp),
        FMath::Lerp(LayerColours[FirstIndex].G, LayerColours[LastIndex].G, Lerp),
        FMath::Lerp(LayerColours[FirstIndex].B, LayerColours[LastIndex].B, Lerp)
    );
}

void ASVONavVolume::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
	FProperty* Property = PropertyChangedEvent.Property;
	const FString PropertyName = Property != nullptr ? Property->GetFName().ToString() : "";

	const TSet<FString> CriticalProperties = {
		"VolumeSize",
		"VoxelSize"
	};
	const TSet<FString> DebugProperties = {
		"bDisplayVolumeBounds",
		"VolumeBoundsColor",
		"bDisplayLayers",
		"bDisplayLeaves",
		"bDisplayLeafOcclusion",
		"bDisplayNeighbourLink",
		"LineScale",
		"LayerColours",
		"LeafOcclusionColour",
		"bDisplayMortonCodes",
		"MortonCodeColour",
		"MortonCodeScale"
	};
	if (CriticalProperties.Contains(PropertyName))
	{
		Initialise();
		DebugDrawOctree();
	}
	else if (DebugProperties.Contains(PropertyName))
	{
		DebugDrawOctree();
	}
}

void ASVONavVolume::PostEditUndo()
{
	Super::PostEditUndo();
	Initialise();
}

void ASVONavVolume::FlushDebugDraw() const
{
	if (!GetWorld()) return;
	FlushPersistentDebugLines(GetWorld());
	FlushDebugStrings(GetWorld());
}

void ASVONavVolume::AddDebugNavPath(const FSVONavDebugPath DebugPath)
{
	DebugPaths.Add(DebugPath);
	RequestOctreeDebugDraw();
}

void ASVONavVolume::RequestOctreeDebugDraw()
{
	bDebugDrawRequested = true;

	DebugDrawOctree();
	DebugDrawOctree_Hie();
	bDebugDrawRequested = false;
}

void ASVONavVolume::DebugDrawOctree()
{
	GetWorld()->PersistentLineBatcher->SetComponentTickEnabled(false);
	FlushDebugDraw();
	if (OctreeValid())
	{
		for (int32 I = 0; I < Octree.Layers.Num(); I++)
		{
			for (int32 J = 0; J < Octree.Layers[I].Num(); J++)
			{
				FVector NodeLocation;
				GetNodeLocation(I, Octree.Layers[I][J].MortonCode, NodeLocation);
				if (I == 0 && bDisplayLeaves || I > 0 && bDisplayLayers)
				{
					DebugDrawVoxel(NodeLocation, FVector(VoxelHalfSizes[I]), GetLayerColour(I));
				}
				if (bDisplayMortonCodes)
				{
					DebugDrawMortonCode(NodeLocation,
					                    FString::FromInt(I) + ":" + FString::FromInt(Octree.Layers[I][J].MortonCode),
					                    MortonCodeColour);
				}
			}
		}

		// Links must be rebuilt to draw Link adjacency
		if (bDisplayNeighbourLink)
		{
			DebugLinks.Empty();
			for (int32 I = NumLayers - 2; I >= 0; I--) BuildLinks(I);
			DebugDrawNeighbourLink();
		}

		if (bDisplayLeafOcclusion) DebugDrawLeafOcclusion();
	}
}

void ASVONavVolume::DebugDrawVolume() const
{
	if (!GetWorld()) return;
	if (bDisplayVolumeBounds)
	{
		const FBox Box = GetBoundingBox();
		DebugDrawBoundsMesh(Box, VolumeBoundsColour);
	}
}

void ASVONavVolume::DebugDrawVoxel(FVector Location, FVector Extent, FColor Colour) const
{
	if (!InDebugRange(Location)) return;
	DrawDebugBox(GetWorld(), Location, Extent, FQuat::Identity, Colour, true, -1.f, 0, LineScale);
}

void ASVONavVolume::DebugDrawSphere(const FVector Location, const float Radius, const FColor Colour) const
{
	if (!InDebugRange(Location)) return;
	DrawDebugSphere(GetWorld(), Location, Radius, 12, Colour, true, -1.f, 0, LineScale);
}

void ASVONavVolume::DebugDrawMortonCode(FVector Location, FString String, FColor Colour) const
{
	if (!InDebugRange(Location)) return;

	FVector Start, End;
	const float ScaleFactor = VolumeSize * 0.001f * MortonCodeScale;
	const FVector Scale = FVector(1.f, 3.0f, 6.0f) * ScaleFactor;
	const float Tracking = 6.f * ScaleFactor;

	// Draw the background box
	const FVector Extents = FVector(
		0, (String.Len() * Scale.Y + (String.Len() - 1) * Tracking) * 0.5f + ScaleFactor * 4,
		Scale.Z + ScaleFactor * 2);
	const FColor BoxColour = FColor(Colour.R * 0.25f, Colour.G * 0.25f, Colour.B * 0.25f, Colour.A);
	const TArray<FVector> Vertices = {
		{Location.X + 1.f, Location.Y - Extents.Y, Location.Z + Extents.Z},
		{Location.X + 1.f, Location.Y + Extents.Y, Location.Z + Extents.Z},
		{Location.X + 1.f, Location.Y + Extents.Y, Location.Z - Extents.Z},
		{Location.X + 1.f, Location.Y - Extents.Y, Location.Z - Extents.Z}
	};
	const TArray<int32> Indices = {0, 1, 2, 0, 2, 3};
	DrawDebugMesh(GetWorld(), Vertices, Indices, BoxColour, true, -1.0, 0);

	// Draw the morton code string with drawn lines, like a calculator interface
	for (int32 I = 0; I < String.Len(); I++)
	{
		TArray<bool> Layout;
		switch (String[I])
		{
		case 0x30: Layout = {true, true, true, false, true, true, true};
			break;
		case 0x31: Layout = {false, false, true, false, false, true, false};
			break;
		case 0x32: Layout = {true, false, true, true, true, false, true};
			break;
		case 0x33: Layout = {true, false, true, true, false, true, true};
			break;
		case 0x34: Layout = {false, true, true, true, false, true, false};
			break;
		case 0x35: Layout = {true, true, false, true, false, true, true};
			break;
		case 0x36: Layout = {true, true, false, true, true, true, true};
			break;
		case 0x37: Layout = {true, false, true, false, false, true, false};
			break;
		case 0x38: Layout = {true, true, true, true, true, true, true};
			break;
		case 0x39: Layout = {true, true, true, true, false, true, true};
			break;
		default: Layout = {false, false, false, true, false, false, false};
			break;
		}
		for (int32 J = 0; J < 7; J++)
		{
			if (!Layout[J]) continue;
			switch (J)
			{
			case 0: Start = FVector(0, -1, 1);
				End = FVector(0, 1, 1);
				break;
			case 1: Start = FVector(0, -1, 1);
				End = FVector(0, -1, 0);
				break;
			case 2: Start = FVector(0, 1, 1);
				End = FVector(0, 1, 0);
				break;
			case 3: Start = FVector(0, -1, 0);
				End = FVector(0, 1, 0);
				break;
			case 4: Start = FVector(0, -1, 0);
				End = FVector(0, -1, -1);
				break;
			case 5: Start = FVector(0, 1, 0);
				End = FVector(0, 1, -1);
				break;
			case 6: Start = FVector(0, -1, -1);
				End = FVector(0, 1, -1);
				break;
			default: break;
			}
			const float YOffset = (String.Len() * Scale.Y + (String.Len() - 1) * Tracking) * -0.5f + (Scale.Y + Tracking
			) * I;
			Start = Start * Scale + FVector(0, YOffset, 0) + Location;
			End = End * Scale + FVector(0, YOffset, 0) + Location;
			DrawDebugLine(GetWorld(), Start, End, Colour, true, -1.0f, 0, ScaleFactor);
		}
	}
}

void ASVONavVolume::DebugDrawLeafOcclusion()
{
	for (uint_fast32_t I = 0; I < static_cast<uint_fast32_t>(Octree.Leaves.Num()); I++)
	{
		for (uint8 J = 0; J < 64; J++)
		{
			if (Octree.Leaves[I].GetSubNode(J))
			{
				const FSVONavLink Link{0, I, J};
				FVector NodeLocation;
				GetNodeLocation(Link, NodeLocation);
				DebugDrawVoxel(NodeLocation, FVector(VoxelHalfSizes[0] * 0.25f), LeafOcclusionColour);
			}
		}
	}
}

void ASVONavVolume::DebugDrawNeighbourLink() const
{
	if (!GetWorld()) return;
	for (auto& Link : DebugLinks)
	{
		DrawDebugLine(GetWorld(), Link.Start, Link.End, GetLayerColour(Link.LayerIndex), true, -1, 0, LineScale);
	}
}

void ASVONavVolume::DebugDrawBoundsMesh(FBox Box, FColor Colour) const
{
	const TArray<FVector> Vertices = {
		{Box.Min.X, Box.Min.Y, Box.Min.Z}, {Box.Max.X, Box.Min.Y, Box.Min.Z}, {Box.Max.X, Box.Min.Y, Box.Max.Z},
		{Box.Min.X, Box.Min.Y, Box.Max.Z},
		{Box.Min.X, Box.Max.Y, Box.Min.Z}, {Box.Max.X, Box.Max.Y, Box.Min.Z}, {Box.Max.X, Box.Max.Y, Box.Max.Z},
		{Box.Min.X, Box.Max.Y, Box.Max.Z}
	};
	const TArray<int32> Indices = {
		0, 1, 2, 0, 2, 3, 5, 4, 7, 5, 7, 6, 3, 2, 6, 3, 6, 7, 5, 4, 0, 5, 0, 1, 0, 4, 7, 0, 7, 3, 5, 1, 2, 5, 2, 6
	};
	DrawDebugMesh(GetWorld(), Vertices, Indices, Colour, true, -1.0, 0);
}

void ASVONavVolume::EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown,
                                           bool bCtrlDown)
{
	Super::EditorApplyTranslation(DeltaTranslation, bAltDown, bShiftDown, bCtrlDown);
	Initialise();
}

void ASVONavVolume::EditorApplyRotation(const FRotator& DeltaRotation, bool bAltDown, bool bShiftDown, bool bCtrlDown)
{
	Super::EditorApplyRotation(DeltaRotation, bAltDown, bShiftDown, bCtrlDown);
	Initialise();
}

void ASVONavVolume::EditorApplyScale(const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown,
                                     bool bShiftDown, bool bCtrlDown)
{
	Super::EditorApplyScale(DeltaScale, PivotLocation, bAltDown, bShiftDown, bCtrlDown);
	Initialise();
}

#endif

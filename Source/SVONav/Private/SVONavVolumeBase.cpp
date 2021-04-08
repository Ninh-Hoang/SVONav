#include "SVONavVolumeBase.h"
#include "chrono"
#include "DrawDebugHelpers.h"
#include "SVONavUpdateOctreeTask.h"
#include "Components/BrushComponent.h"
#include "Async/ParallelFor.h"
#include "Layers/LayersSubsystem.h"
#include "Async/Async.h"
#include "Builders/CubeBuilder.h"
using namespace std::chrono;

ASVONavVolumeBase::ASVONavVolumeBase(const FObjectInitializer& ObjectInitializer)
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

void ASVONavVolumeBase::UpdateOctree()
{
#if WITH_EDITOR
	const auto UpdateStartTime = high_resolution_clock::now();
#endif

	Octree = CachedOctree;
}

bool ASVONavVolumeBase::BuildOctree()
{
	//init setup
	Initialise();

#if WITH_EDITOR
	const auto StartTime = high_resolution_clock::now();
#endif

	InternalBuildOctree();
	
#if WITH_EDITOR
	const float Duration = std::chrono::duration_cast<milliseconds>(high_resolution_clock::now() - StartTime).count() /
        1000.0f;

	// Octree info
	int32 NumNodes = 0;
	for (int32 I = 0; I < NumLayers; I++) NumNodes += Octree.Layers[I].Num();

	UE_LOG(LogTemp, Display, TEXT("Generation Time : %f seconds"), Duration);
	UE_LOG(LogTemp, Display, TEXT("Desired Volume Size : %fcm"), VolumeSize);
	UE_LOG(LogTemp, Display, TEXT("Actual Volume Size : %fcm"), ActualVolumeSize);
	UE_LOG(LogTemp, Display, TEXT("Voxel Size : %fcm"), VoxelSize);
	UE_LOG(LogTemp, Display, TEXT("Voxel Exponent: %i"), VoxelExponent);
	UE_LOG(LogTemp, Display, TEXT("Total Layers : %i"), NumLayers);
	UE_LOG(LogTemp, Display, TEXT("Total Nodes : %i"), NumNodes);
	UE_LOG(LogTemp, Display, TEXT("Total Leaves : %i"), Octree.Leaves.Num());
	UE_LOG(LogTemp, Display, TEXT("Total Octree Bytes : %i"), NumBytes);

	DebugDrawOctree();
#endif
	for (int32 i = 0; i < NumLayer; i ++)
	{
		UE_LOG(LogTemp, Warning, TEXT("Layer %i num: %i"), i, Octree.Layers[i].Num());
	}
	return true;
}

void ASVONavVolumeBase::InternalBuildOctree()
{
}

void ASVONavVolumeBase::GetMortonVoxel(const FVector& Location, int32 LayerIndex, FIntVector& MortonLocation) const
{
	const FVector LocationLocal = Location - (VolumeOrigin - VolumeExtent);
	const float Size = VoxelHalfSizes[LayerIndex] * 2;
	MortonLocation.X = FMath::FloorToInt(LocationLocal.X / Size);
	MortonLocation.Y = FMath::FloorToInt(LocationLocal.Y / Size);
	MortonLocation.Z = FMath::FloorToInt(LocationLocal.Z / Size);
}

bool ASVONavVolumeBase::FindLink(layerindex_t LayerIndex, int32 NodeIndex, uint8 Direction, FSVONavLink& Link,
                                 const FVector& NodeLocation)
{
	TArray<FSVONavNode>& LayerNodes = Octree.Layers[LayerIndex];
	FSVONavNode& TargetNode = LayerNodes[NodeIndex];

	uint_fast32_t X, Y, Z;
	morton3D_64_decode(TargetNode.MortonCode, X, Y, Z);

	FIntVector S(static_cast<int32>(X), static_cast<int32>(Y), static_cast<int32>(Z));
	S += Directions[Direction];
	//do check range here

	X = S.X;
	Y = S.Y;
	Z = S.Z;

	const uint_fast64_t AdjacentCode = morton3D_64_encode(X, Y, Z);

	int32 NeighbourNodeIndex;
	if (GetNodeIndex_Hie(LayerIndex, AdjacentCode, NeighbourNodeIndex))
	{
		Link.SetLayerIndex(LayerIndex);
		Link.SetNodeIndex(NeighbourNodeIndex);

#if WITH_EDITOR
		if (LayerIndex != 0)
		{
			FVector NeighbourLocation;
			GetNodeLocation_Hie(LayerIndex, LayerNodes[NeighbourNodeIndex].MortonCode, NeighbourLocation);
			DebugLinks.Add(FSVONavDebugLink(NodeLocation, NeighbourLocation, LayerIndex));
		}
#endif

		return true;
	}
	Link.Invalidate();
	return false;
}

void ASVONavVolumeBase::UpdateTaskComplete()
{
	UnlockOctree();

#if WITH_EDITOR
	// Run the debug draw on the game thread
	AsyncTask(ENamedThreads::GameThread, [=]()
	{
		DebugDrawOctree();
	});
#endif
}

void ASVONavVolumeBase::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
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

void ASVONavVolumeBase::PostEditUndo()
{
	Super::PostEditUndo();
	Initialise();
}

void ASVONavVolumeBase::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
}

void ASVONavVolumeBase::BeginPlay()
{
	CachedOctree = Octree;
	OnUpdateComplete.BindUObject(this, &ASVONavVolumeBase::UpdateTaskComplete);
	SetActorTickInterval(TickInterval);
}

void ASVONavVolumeBase::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!bOctreeLocked)
	{
		if (bUpdateRequested)
		{
			// Prevent any further requests until update task is complete
			LockOctree();

			// Execute UpdateOctree as background task
			(new FAutoDeleteAsyncTask<FSVONavUpdateOctreeTask>(this, OnUpdateComplete))->StartBackgroundTask();

			// Update complete
			bUpdateRequested = false;

#if WITH_EDITOR
		}
		else if (bDebugDrawRequested)
		{
			DebugDrawOctree();
			bDebugDrawRequested = false;
#endif
		}
	}
}

void ASVONavVolumeBase::PostRegisterAllComponents()
{
	Super::PostRegisterAllComponents();
}

void ASVONavVolumeBase::PostUnregisterAllComponents()
{
	Super::PostUnregisterAllComponents();
}

void ASVONavVolumeBase::EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown,
                                               bool bCtrlDown)
{
	Super::EditorApplyTranslation(DeltaTranslation, bAltDown, bShiftDown, bCtrlDown);
	Initialise();
}

void ASVONavVolumeBase::EditorApplyRotation(const FRotator& DeltaRotation, bool bAltDown, bool bShiftDown,
                                            bool bCtrlDown)
{
	Super::EditorApplyRotation(DeltaRotation, bAltDown, bShiftDown, bCtrlDown);
	Initialise();
}

void ASVONavVolumeBase::EditorApplyScale(const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown,
                                         bool bShiftDown, bool bCtrlDown)
{
	Super::EditorApplyScale(DeltaScale, PivotLocation, bAltDown, bShiftDown, bCtrlDown);
	Initialise();
}

void ASVONavVolumeBase::UpdateVolume()
{
	// Calculate the nearest integer exponent to fit the voxel size perfectly within the volume extents
	VoxelExponent = FMath::RoundToInt(FMath::Log2(VolumeSize / (VoxelSize)));

	NumLayers = VoxelExponent + 1;

	// Build a list of voxel half-scale sizes for each layer
	VoxelHalfSizes.Reset();
	VoxelHalfSizes.Reserve(NumLayers);
	for (int32 I = 0; I < NumLayers; I++) VoxelHalfSizes.Add(GetVoxelScale(I) * 0.5f);

	ActualVolumeSize = GetActualVolumeSize();
	UCubeBuilder* CubeBuilder = Cast<UCubeBuilder>(GEditor->FindBrushBuilder(UCubeBuilder::StaticClass()));
	CubeBuilder->X = ActualVolumeSize;
	CubeBuilder->Y = ActualVolumeSize;
	CubeBuilder->Z = ActualVolumeSize;
	CubeBuilder->Build(GetWorld(), this);

	const FBox Bounds = GetComponentsBoundingBox(true);
	Bounds.GetCenterAndExtents(VolumeOrigin, VolumeExtent);
}

void ASVONavVolumeBase::Serialize(FArchive& Ar)
{
	Super::Serialize(Ar);
	Ar << Octree;
	Ar << VoxelHalfSizes;
	Ar << VolumeExtent;
	NumBytes = Octree.GetSize();
}

void ASVONavVolumeBase::Initialise()
{
	Octree.Reset();
	BlockedIndices.Empty();
	NumBytes = 0;

#if WITH_EDITOR
	FlushDebugDraw();
#endif

	UpdateVolume();
}

void ASVONavVolumeBase::GetVolumeExtents(const FVector& Location, int32 LayerIndex, FIntVector& Extents) const
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

FBox ASVONavVolumeBase::GetBoundingBox() const
{
	const FBoxSphereBounds Bounds = GetBrushComponent()->CalcBounds(ActorToWorld());
	return Bounds.GetBox();
}

bool ASVONavVolumeBase::GetLink(const FVector& Location, FSVONavLink& Link)
{
	if (!IsWithinBounds(Location)) return false;

	int32 LayerIndex = Octree.Layers.Num() - 2;
	while (LayerIndex >= 0)
	{
		const TArray<FSVONavNode>& Layer = GetLayer_Hie(LayerIndex);
		FIntVector Voxel;
		GetMortonVoxel(Location, LayerIndex, Voxel);
		const uint_fast64_t MortonCode = morton3D_64_encode(Voxel.X, Voxel.Y, Voxel.Z);
		int32 NodeIndex;
		if (GetNodeIndex_Hie(LayerIndex, MortonCode, NodeIndex))
		{
			const FSVONavNode Node = Layer[NodeIndex];

			if (!Node.HasChildren())
			{
				Link.SetLayerIndex(LayerIndex);
				Link.SetNodeIndex(NodeIndex);
				Link.SetSubNodeIndex(0);
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

void ASVONavVolumeBase::FlushDebugDraw() const
{
	if (!GetWorld()) return;
	FlushPersistentDebugLines(GetWorld());
	FlushDebugStrings(GetWorld());
}

void ASVONavVolumeBase::AddDebugNavPath(const FSVONavDebugPath DebugPath)
{
}

void ASVONavVolumeBase::RequestOctreeDebugDraw()
{
	bDebugDrawRequested = true;
	DebugDrawOctree();
	bDebugDrawRequested = false;
}

void ASVONavVolumeBase::DebugDrawOctree()
{
}

int32 ASVONavVolumeBase::GetSegmentNodeCount_Hie(layerindex_t LayerIndex) const
{
	return FMath::Pow(2, VoxelExponent - LayerIndex);
}

bool ASVONavVolumeBase::GetNodeIndex_Hie(layerindex_t LayerIndex, uint_fast64_t NodeMortonCode, int32& NodeIndex) const
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

float ASVONavVolumeBase::GetVoxelScale(uint8 LayerIndex) const
{
	return VolumeExtent.X / FMath::Pow(2.0f, VoxelExponent) * FMath::Pow(2.0f, LayerIndex + 1);
}

bool ASVONavVolumeBase::GetNodeLocation_Hie(uint8 LayerIndex, uint_fast64_t MortonCode, FVector& Location) const
{
	const float Scale = VoxelHalfSizes[LayerIndex] * 2;
	uint_fast32_t X, Y, Z;
	morton3D_64_decode(MortonCode, X, Y, Z);
	Location = VolumeOrigin - VolumeExtent + Scale * FVector(X, Y, Z) + FVector(Scale * 0.5f);
	return true;
}

bool ASVONavVolumeBase::LinkNodeIsValid(const FSVONavLink& Link) const
{
	if (static_cast<int32>(Link.LayerIndex) >= Octree.Layers.Num()) return false;
	return Link.IsValid() && static_cast<int32>(Link.NodeIndex) < Octree.Layers[Link.LayerIndex].Num();
}

void ASVONavVolumeBase::GetNeighbourLinks(const FSVONavLink& Link, TArray<FSVONavLink>& NeighbourLinks) const
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

int32 ASVONavVolumeBase::GetLayerNodeCount_Hie(layerindex_t LayerIndex) const
{
	return FMath::Pow(8, VoxelExponent - LayerIndex);
}

const FSVONavNode& ASVONavVolumeBase::GetNode(const FSVONavLink& Link) const
{
	return Octree.Layers[Link.LayerIndex][Link.NodeIndex];
}

FColor ASVONavVolumeBase::GetLayerColour(const int32 LayerIndex) const
{
	const float Ratio = LayerColours.Num() / static_cast<float>(NumLayer) * LayerIndex;
	const int32 FirstIndex = FMath::FloorToInt(Ratio);
	const int32 LastIndex = FMath::Min(FMath::CeilToInt(Ratio), LayerColours.Num() - 1);
	const float Lerp = FMath::Fmod(Ratio, 1);
	return FColor(
		FMath::Lerp(LayerColours[FirstIndex].R, LayerColours[LastIndex].R, Lerp),
		FMath::Lerp(LayerColours[FirstIndex].G, LayerColours[LastIndex].G, Lerp),
		FMath::Lerp(LayerColours[FirstIndex].B, LayerColours[LastIndex].B, Lerp)
	);
}

bool ASVONavVolumeBase::IsBlocked(const FVector& Location, float Size) const
{
	return GetWorld()->OverlapBlockingTestByChannel(
		Location,
		FQuat::Identity,
		CollisionChannel,
		FCollisionShape::MakeBox(FVector(Size + Clearance)),
		CollisionQueryParams
	);
}

bool ASVONavVolumeBase::IsBlocked(const FVector& Location, float Size, TArray<FOverlapResult>& OverlapResults) const
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

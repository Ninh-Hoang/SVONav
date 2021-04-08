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
	if(Octree.Layers.Num() > 0)
	{
		for (int32 I = 0; I < Octree.Layers.Num(); I++) NumNodes += Octree.Layers[I].Num();
	}

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
			GetNodeLocation(LayerIndex, LayerNodes[NeighbourNodeIndex].MortonCode, NeighbourLocation);
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

int32 ASVONavVolumeBase::GetSegmentNodeCount(layerindex_t LayerIndex) const
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

bool ASVONavVolumeBase::GetNodeLocation(const FSVONavLink& Link, FVector& Location)
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

float ASVONavVolumeBase::GetVoxelScale(uint8 LayerIndex) const
{
	return VolumeExtent.X / FMath::Pow(2.0f, VoxelExponent) * FMath::Pow(2.0f, LayerIndex + 1);
}

bool ASVONavVolumeBase::GetNodeLocation(uint8 LayerIndex, uint_fast64_t MortonCode, FVector& Location) const
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

int32 ASVONavVolumeBase::GetLayerNodeCount(layerindex_t LayerIndex) const
{
	return FMath::Pow(8, VoxelExponent - LayerIndex);
}

const FSVONavNode& ASVONavVolumeBase::GetNode(const FSVONavLink& Link) const
{
	return Octree.Layers[Link.LayerIndex][Link.NodeIndex];
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

void ASVONavVolumeBase::Serialize(FArchive& Ar)
{
	Super::Serialize(Ar);
	Ar << Octree;
	Ar << VoxelHalfSizes;
	Ar << VolumeExtent;
	NumBytes = Octree.GetSize();
}

#if WITH_EDITOR

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

void ASVONavVolumeBase::RegenerateLinkForDebug()
{
	
}

void ASVONavVolumeBase::DebugDrawOctree()
{
	DebugLinks.Empty();
	if (bDisplayLayers)
	{
		for (int32 a = 0; a < NumLayer; a++)
		{
			for (int32 i = 0; i < Octree.Layers[a].Num(); i ++)
			{
				FSVONavNode& Node = Octree.Layers[a][i];
				FVector NodeLocation;
				GetNodeLocation(a, Node.MortonCode, NodeLocation);
				DebugDrawVoxel(NodeLocation, FVector(VoxelHalfSizes[a]), GetLayerColour(a));
			}
		}
	}
	if (bDisplayNeighbourLink)
	{
		RegenerateLinkForDebug();
	}
	for(auto& DebugVoxel : DebugVoxelList)
	{
		if(DebugVoxel.Layer >=  Octree.Layers.Num()) continue;
		if(DebugVoxel.Index >= Octree.Layers[DebugVoxel.Layer].Num()) continue;
		
		FSVONavNode& Node = Octree.Layers[DebugVoxel.Layer][DebugVoxel.Index];
		FVector NodeLocation, ParentLocation;
		GetNodeLocation(DebugVoxel.Layer, Node.MortonCode, NodeLocation);
		DebugDrawVoxel(NodeLocation, FVector(VoxelHalfSizes[DebugVoxel.Layer]), GetLayerColour(DebugVoxel.Layer));
		if(Node.Parent.IsValid())
		{
			GetNodeLocation(DebugVoxel.Layer, Node.MortonCode, NodeLocation);
			DebugLinks.Add(FSVONavDebugLink(NodeLocation, ParentLocation,Node.Parent.LayerIndex));
			UE_LOG(LogTemp, Warning, TEXT("Parent Layer: %i, Parent Index: %i"), Node.Parent.GetLayerIndex(), Node.Parent.GetNodeIndex());
		}
		for(auto& Link : Node.Neighbours)
		{
			if(Link.IsValid())
			{
				FVector NeighbourLocation;
				GetNodeLocation(Link.GetLayerIndex(), GetNode(Link).MortonCode, NeighbourLocation);
				DebugLinks.Add(FSVONavDebugLink(NodeLocation, NeighbourLocation, Link.GetLayerIndex()));
				UE_LOG(LogTemp, Warning, TEXT("Neighbour Layer: %i, Neighbour Index: %i"), Link.GetLayerIndex(), Link.GetNodeIndex());
			}
		}
		for(auto& Link : Node.Childs)
		{
			if(Link.IsValid())
			{
				FVector ChildLocation;
				GetNodeLocation(Link.GetLayerIndex(), GetNode(Link).MortonCode, ChildLocation);
				DebugLinks.Add(FSVONavDebugLink(NodeLocation, ChildLocation, Link.GetLayerIndex()));
				UE_LOG(LogTemp, Warning, TEXT("Child Layer: %i, Child Index: %i"), Link.GetLayerIndex(), Link.GetNodeIndex());
			}
		}
	}
	
	DebugDrawNeighbourLink();

	//check out of bound voxel
	for(int32 R = 0; R < Octree.Layers.Num(); R++)
	{
		for(int32 V = 0; V < Octree.Layers[R].Num(); V++)
		{
			uint_fast32_t X, Y, Z;
			morton3D_64_decode(Octree.Layers[R][V].MortonCode, X, Y, Z);
			int32 C = GetSegmentNodeCount(R);
			FIntVector S(static_cast<int32>(X), static_cast<int32>(Y), static_cast<int32>(Z));
			if(S.X >= C || S.Y >= C || S.Z >= C)
			{
				UE_LOG(LogTemp, Warning, TEXT("Node Layer: %i, Node Index: %i, MortonCode: %i"), R, V, Octree.Layers[R][V].MortonCode);	
			}
		}
	}
}

void ASVONavVolumeBase::DebugDrawVolume() const
{
	if (!GetWorld()) return;
	if (bDisplayVolumeBounds)
	{
		const FBox Box = GetBoundingBox();
		DebugDrawBoundsMesh(Box, VolumeBoundsColour);
	}
}

void ASVONavVolumeBase::DebugDrawVoxel(FVector Location, FVector Extent, FColor Colour) const
{
	if (!InDebugRange(Location)) return;
	DrawDebugBox(GetWorld(), Location, Extent, FQuat::Identity, Colour, true, -1.f, 0, LineScale);
}

void ASVONavVolumeBase::DebugDrawSphere(const FVector Location, const float Radius, const FColor Colour) const
{
	if (!InDebugRange(Location)) return;
	DrawDebugSphere(GetWorld(), Location, Radius, 12, Colour, true, -1.f, 0, LineScale);
}

void ASVONavVolumeBase::DebugDrawMortonCode(FVector Location, FString String, FColor Colour) const
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

bool ASVONavVolumeBase::InDebugRange(FVector Location) const
{
	if (!GetWorld()) return true;
	if (GetWorld()->ViewLocationsRenderedLastFrame.Num() == 0) return true;
	return FVector::Dist(GetWorld()->ViewLocationsRenderedLastFrame[0], Location) < DebugDistance;
}

void ASVONavVolumeBase::DebugDrawLeafOcclusion()
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

void ASVONavVolumeBase::DebugDrawNeighbourLink() const
{
	if (!GetWorld()) return;
	for (auto& Link : DebugLinks)
	{
		DrawDebugLine(GetWorld(), Link.Start, Link.End, GetLayerColour(Link.LayerIndex), true, -1, 0, LineScale);
	}
}

void ASVONavVolumeBase::DebugDrawBoundsMesh(FBox Box, FColor Colour) const
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

#endif
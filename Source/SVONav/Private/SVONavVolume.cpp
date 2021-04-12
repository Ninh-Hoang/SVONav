// Fill out your copyright notice in the Description page of Project Settings.

#include "SVONavVolume.h"
#include "Builders/CubeBuilder.h"

ASVONavVolume::ASVONavVolume(const FObjectInitializer& ObjectInitializer)
	: Super(ObjectInitializer)
{
}

void ASVONavVolume::InternalBuildOctree()
{
	InitRasterize();
	Octree.Leaves.AddDefaulted(BlockedIndices[0].Num() * 8 * 0.25f);
	
	for (int32 I = 0; I < NumLayers; I++) RasterizeLayer(I);
	
	for (int32 I = NumLayers - 2; I >= 0; I--) BuildLinks(I);
}

void ASVONavVolume::InitRasterize()
{
	BlockedIndices.Emplace();

	for (int32 I = 0; I < GetLayerNodeCount(1); I++)
	{
		FVector Location;
		GetNodeLocation(1, I, Location);
		if (IsBlocked(Location, VoxelHalfSizes[1]))
		{
			BlockedIndices[0].Add(I);
		}
	}

	for (int32 I = 0; I < VoxelExponent; I++)
	{
		BlockedIndices.Emplace();
		for (uint_fast64_t& MortonCode : BlockedIndices[I])
		{
			BlockedIndices[I + 1].Add(MortonCode >> 3);
		}
	}
}

void ASVONavVolume::RasterizeLayer(uint8 LayerIndex)
{
	Octree.Layers.Emplace();
	int32 LeafIndex = 0;
	if (LayerIndex == 0)
	{
		Octree.Leaves.Reserve(BlockedIndices[0].Num() * 8);
		Octree.Layers[0].Reserve(BlockedIndices[0].Num() * 8);
		const int32 NumNodes = GetLayerNodeCount(0);
		for (int32 I = 0; I < NumNodes; I++)
		{			
			if (BlockedIndices[0].Contains(I >> 3))
			{
				const int32 Index = GetLayer(0).Emplace();
				FSVONavNode& NewNode = Octree.Layers[0][Index];
				NewNode.MortonCode = I;
				FVector NodeLocation;
				GetNodeLocation(0, I, NodeLocation);
				if (IsBlocked(NodeLocation, VoxelHalfSizes[0])) {
					RasterizeLeaf(NodeLocation, LeafIndex);
					NewNode.FirstChild.SetLayerIndex(0);
					NewNode.FirstChild.SetNodeIndex(LeafIndex);
					NewNode.FirstChild.SetSubNodeIndex(0);
					LeafIndex++;
				} else {
					Octree.Leaves.AddDefaulted(1);
					LeafIndex++;
					NewNode.FirstChild.Invalidate();
				}
			}
		}
	} 
	else if (GetLayer(LayerIndex - 1).Num() > 0)
	{
		Octree.Layers[LayerIndex].Reserve(BlockedIndices[LayerIndex].Num() * 8);
		const int32 NumNodes = GetLayerNodeCount(LayerIndex);
		for (int32 I = 0; I < NumNodes; I++)
		{
			if (IsAnyMemberBlocked(LayerIndex, I))
			{
				// Add a node
				const int32 Index = GetLayer(LayerIndex).Emplace();
				FSVONavNode& NewNode = GetLayer(LayerIndex)[Index];
				NewNode.MortonCode = I;
				int32 ChildIndex = 0;
				if (GetNodeIndex(LayerIndex - 1, NewNode.MortonCode << 3, ChildIndex))
				{
					NewNode.FirstChild.SetLayerIndex(LayerIndex - 1);
					NewNode.FirstChild.SetNodeIndex(ChildIndex);
					for (int32 C = 0; C < 8; C++)
					{
						GetLayer(NewNode.FirstChild.GetLayerIndex())[NewNode.FirstChild.GetNodeIndex() + C].Parent.
							SetLayerIndex(LayerIndex);
						GetLayer(NewNode.FirstChild.GetLayerIndex())[NewNode.FirstChild.GetNodeIndex() + C].Parent.
							SetNodeIndex(Index);
					}
				}
				else
				{
					NewNode.FirstChild.Invalidate();
				}
			}
		}
	}
}

void ASVONavVolume::RasterizeLeaf(FVector NodeLocation, int32 LeafIndex)
{
	const FVector Location = NodeLocation - VoxelHalfSizes[0];
	const float VoxelScale = VoxelHalfSizes[0] * 0.5f;
	for (int32 I = 0; I < 64; I++) {
		uint_fast32_t X, Y, Z;
		morton3D_64_decode(I, X, Y, Z);
		const FVector VoxelLocation = Location + FVector(X * VoxelScale, Y * VoxelScale, Z * VoxelScale) + VoxelScale * 0.5f;
		if (LeafIndex >= Octree.Leaves.Num() - 1) Octree.Leaves.AddDefaulted(1);

		if (IsBlocked(VoxelLocation, VoxelScale * 0.5f))
		{
			Octree.Leaves[LeafIndex].SetSubNode(I);
		}
	}
}

void ASVONavVolume::BuildLinks(layerindex_t LayerIndex)
{
	if (Octree.Layers.Num() == 0) return;
	TArray<FSVONavNode>& Layer = GetLayer(LayerIndex);
	for (int32 I = 0; I < Layer.Num(); I++) {
		FSVONavNode& Node = Layer[I];
		FVector NodeLocation;
		GetNodeLocation(LayerIndex, Node.MortonCode, NodeLocation);

		for (int32 Direction = 0; Direction < 6; Direction++) {
			int32 NodeIndex = I;
			FSVONavLink& Edge = Node.Neighbours[Direction];
			layerindex_t CurrentLayer = LayerIndex;
			while (!FindLink(CurrentLayer, NodeIndex, Direction, Edge, NodeLocation) && CurrentLayer < Octree.Layers.Num() - 2) {
				FSVONavLink& ParentEdge = GetLayer(CurrentLayer)[NodeIndex].Parent;
				if (ParentEdge.IsValid()) {
					NodeIndex = ParentEdge.NodeIndex;
					CurrentLayer = ParentEdge.LayerIndex;
				} else {
					CurrentLayer++;
					GetNodeIndex(CurrentLayer, Node.MortonCode >> 3, NodeIndex);
				}
			}
		}
	}
}

bool ASVONavVolume::FindLink(layerindex_t LayerIndex, int32 NodeIndex, uint8 Direction, FSVONavLink& Link,
	const FVector& NodeLocation)
{
	const int32 MaxCoordinate = GetSegmentNodeCount(LayerIndex);
	
	TArray<FSVONavNode>& Layer = GetLayer(LayerIndex);
	FSVONavNode& TargetNode = GetLayer(LayerIndex)[NodeIndex];
	
	uint_fast32_t X, Y, Z;
	morton3D_64_decode(TargetNode.MortonCode, X, Y, Z);
	
	// Create a signed vector from the X Y and Z values
	FIntVector S( static_cast<int32>(X), static_cast<int32>(Y), static_cast<int32>(Z) );
	S += Directions[Direction];
	if (S.X < 0 || S.X >= MaxCoordinate ||
        S.Y < 0 || S.Y >= MaxCoordinate ||
        S.Z < 0 || S.Z >= MaxCoordinate) {
		Link.Invalidate();
		return true;
        }
	X = S.X;
	Y = S.Y;
	Z = S.Z;
	
	const uint_fast64_t AdjacentCode = morton3D_64_encode(X, Y, Z);
	int32 Stop = Layer.Num();
	int32 NodeDelta = 1;
	if (AdjacentCode < TargetNode.MortonCode)
	{
		NodeDelta = -1;
		Stop = -1;
	}

	for (int32 I = NodeIndex + NodeDelta; I != Stop; I += NodeDelta)
	{
		FSVONavNode& Node = Layer[I];
		if (Node.MortonCode == AdjacentCode)
		{
			if (LayerIndex == 0 && 
                Node.HasChildren() && 
                Octree.Leaves[Node.FirstChild.NodeIndex].IsOccluded()) {
				
				Link.Invalidate();
				return true;
                }
			Link.SetLayerIndex(LayerIndex);
			if (I >= Layer.Num() || I < 0) break; 
			Link.SetNodeIndex(I);
			FVector AdjacentLocation;
			GetNodeLocation(LayerIndex, AdjacentCode, AdjacentLocation);
			
#if WITH_EDITOR
			DebugLinks.Add(FSVONavDebugLink(NodeLocation, AdjacentLocation, LayerIndex));
#endif
			
			return true;
		}
		if (NodeDelta == -1 && Node.MortonCode < AdjacentCode || NodeDelta == 1 && Node.MortonCode > AdjacentCode)
		{
			return false;
		}
	}
	return false;
}

void ASVONavVolume::UpdateVolume()
 {
	// Calculate the nearest integer exponent to fit the voxel size perfectly within the volume extents
	VoxelExponent = FMath::RoundToInt(FMath::Log2(VolumeSize / (VoxelSize * 4)));
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
 
void ASVONavVolume::RegenerateLinkForDebug()
{
}
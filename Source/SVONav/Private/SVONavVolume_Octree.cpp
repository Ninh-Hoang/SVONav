#include "SVONavVolume.h"
#include "chrono"
#include "DrawDebugHelpers.h"
#include "Layers/LayersSubsystem.h"
using namespace std::chrono;

bool ASVONavVolume::BuildOctree()
{
	//init setup
	Initialise();


#if WITH_EDITOR
	const auto StartTime = high_resolution_clock::now();
#endif

	// Build the octree
	InitRasterize();
	Octree.Leaves.AddDefaulted(BlockedIndices[0].Num() * 8 * 0.25f);

	//HieOctree.Layers.Emplace();
	HieOctree.Layers.Emplace();
	for (int32 I = 0; I < NumLayers; I++) RasterizeLayer(I);
	for (int32 I = NumLayers - 2; I >= 0; I--) BuildLinks(I);

	// Clean up and cache the octree
	NumBytes = Octree.GetSize() + HieOctree.GetSize();
	CollisionQueryParams.ClearIgnoredActors();
	CachedOctree = Octree;

	BuildHieOctree();

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
	DebugDrawOctree_Hie();
#endif
	for (int32 i = 0; i < NumLayer_Hie; i ++)
	{
		UE_LOG(LogTemp, Warning, TEXT("Layer %i num: %i"), i, HieOctree.Layers[i].Num());
	}
	return true;
}

void ASVONavVolume::DebugDrawOctree_Hie()
{
	if (bDisplayLayers)
	{
		for (int32 a = 0; a < NumLayer_Hie; a++)
		{
			for (int32 i = 0; i < HieOctree.Layers[a].Num(); i ++)
			{
				FSVONavNode& Node = HieOctree.Layers[a][i];
				FVector NodeLocation;
				GetNodeLocation_Hie(a, Node.MortonCode, NodeLocation);
				DebugDrawVoxel(NodeLocation, FVector(VoxelHalfSizes_Hie[a]), GetLayerColour_Hie(a));
			}
		}
	}
	
	DebugLinks.Empty();
	if (bDisplayNeighbourLink)
	{
		for (int32 i = 1; i < NumLayer_Hie; i++)
		{
			BuildSecondsLinks_Hie(i);
		}
	}
	
	for(auto& DebugVoxel : DebugVoxelList)
	{
		FSVONavNode& Node = HieOctree.Layers[DebugVoxel.Layer][DebugVoxel.Index];
		FVector NodeLocation, ParentLocation;
		GetNodeLocation_Hie(DebugVoxel.Layer, Node.MortonCode, NodeLocation);
		DebugDrawVoxel(NodeLocation, FVector(VoxelHalfSizes_Hie[DebugVoxel.Layer]), GetLayerColour_Hie(DebugVoxel.Layer));
		if(Node.Parent.IsValid())
		{
			GetNodeLocation_Hie(DebugVoxel.Layer, Node.MortonCode, NodeLocation);
			DebugLinks.Add(FSVONavDebugLink(NodeLocation, ParentLocation,Node.Parent.LayerIndex));
			UE_LOG(LogTemp, Warning, TEXT("Parent Layer: %i, Parent Index: %i"), Node.Parent.GetLayerIndex(), Node.Parent.GetNodeIndex());
		}
		for(auto& Link : Node.Neighbours)
		{
			if(Link.IsValid())
			{
				FVector NeighbourLocation;
				GetNodeLocation_Hie(Link.GetLayerIndex(), GetNode_Hie(Link).MortonCode, NeighbourLocation);
				DebugLinks.Add(FSVONavDebugLink(NodeLocation, NeighbourLocation, Link.GetLayerIndex()));
				UE_LOG(LogTemp, Warning, TEXT("Neighbour Layer: %i, Neighbour Index: %i"), Link.GetLayerIndex(), Link.GetNodeIndex());
			}
		}
		for(auto& Link : Node.Childs)
		{
			if(Link.IsValid())
			{
				FVector ChildLocation;
				GetNodeLocation_Hie(Link.GetLayerIndex(), GetNode_Hie(Link).MortonCode, ChildLocation);
				DebugLinks.Add(FSVONavDebugLink(NodeLocation, ChildLocation, Link.GetLayerIndex()));
				UE_LOG(LogTemp, Warning, TEXT("Child Layer: %i, Child Index: %i"), Link.GetLayerIndex(), Link.GetNodeIndex());
			}
		}
	}

	/*int32 L = 4;
	for (int32 i = 0; i < HieOctree.Layers[L].Num(); i++)
	{
		for(int32 E = 0; E < 8; E++)
		{
			FSVONavLink& ChildLink = HieOctree.Layers[L][i].Childs[E];
			if(ChildLink.IsValid())
			{
				FVector ParentLocation, ChildLocation;
				GetNodeLocation_Hie(L, HieOctree.Layers[L][i].MortonCode, ParentLocation);
				GetNodeLocation_Hie(ChildLink.GetLayerIndex(), GetNode_Hie(ChildLink).MortonCode, ChildLocation);
				DebugLinks.Add(FSVONavDebugLink(ParentLocation, ChildLocation, L));
			}
		}
	}*/
	/*for (int32 i = 0; i < HieOctree.Layers[L].Num(); i++)
	{
		FSVONavLink& ChildLink = HieOctree.Layers[L][i].Parent;
		if(ChildLink.IsValid())
		{
			FVector ParentLocation, ChildLocation;
			GetNodeLocation_Hie(L, HieOctree.Layers[L][i].MortonCode, ParentLocation);
			GetNodeLocation_Hie(ChildLink.GetLayerIndex(), GetNode_Hie(ChildLink).MortonCode, ChildLocation);
			DebugLinks.Add(FSVONavDebugLink(ParentLocation, ChildLocation, L));
		}
	}*/
	UE_LOG(LogTemp, Warning, TEXT("DebugLink Count: %i"), DebugLinks.Num());
	if (!GetWorld()) return;
	for (auto& Link : DebugLinks)
	{
		DrawDebugLine(GetWorld(), Link.Start, Link.End, GetLayerColour_Hie(Link.LayerIndex), true, -1, 0,
                      LineScale);
	}

	/*for(int32 I = 0; I < NumLayer_Hie; I++)
	{
		if(HierarchyStartIndex[I] < HieOctree.Layers[I].Num()-1)
		{
			for(int32 C = HierarchyStartIndex[I]; C < HieOctree.Layers[I].Num(); C++)
			{
				FVector NodeLocation;
				GetNodeLocation_Hie(I, HieOctree.Layers[I][C].MortonCode, NodeLocation);
				DebugDrawVoxel(NodeLocation, FVector(VoxelHalfSizes_Hie[I]), GetLayerColour_Hie(I));
			}
		}
	}*/
	for(int32 R = 0; R < HieOctree.Layers.Num(); R++)
	{
		for(int32 V = 0; V < HieOctree.Layers[R].Num(); V++)
		{
			uint_fast32_t X, Y, Z;
			morton3D_64_decode(HieOctree.Layers[R][V].MortonCode, X, Y, Z);
			int32 C = GetSegmentNodeCount_Hie(R);
			FIntVector S(static_cast<int32>(X), static_cast<int32>(Y), static_cast<int32>(Z));
			if(S.X >= C || S.Y >= C || S.Z >= C)
			{
				UE_LOG(LogTemp, Warning, TEXT("Node Layer: %i, Node Index: %i, MortonCode: %i"), R, V, HieOctree.Layers[R][V].MortonCode);	
				//UE_LOG(LogTemp, Warning, TEXT("Node Layer: %i, Node Index: %i, MortonCode: %i, %i, %i"),R, V, X, Y, Z);	
			}
		}
	}

	/*for(int32 R = 0; R < HieOctree.Layers[4].Num(); R++)
	{
		UE_LOG(LogTemp, Warning, TEXT("ChildNum: %i"),HieOctree.Layers[4][R].GetChildNum());	
	}*/

	/*FVector NodeLocation;
	mortoncode_t Code = morton3D_64_encode(0,1,0);
	GetNodeLocation_Hie(0, Code, NodeLocation);
	DebugDrawVoxel(NodeLocation, FVector(100), FColor::Green);*/
	/*for (int32 i = 0; i < HieOctree.Layers[0].Num(); i ++)
	{
		FSVONavNode& Node1 = HieOctree.Layers[0][i];
		FVector NodeLocation1;
		GetNodeLocation_Hie(0, Node1.MortonCode, NodeLocation1);
		DebugDrawVoxel(NodeLocation1, FVector(100), FColor::Green);
	}*/
	/*FVector A;
	GetNodeLocation_Hie(0, HieOctree.Layers[0][0].MortonCode, A);
	DebugDrawVoxel(A, FVector(100), FColor::Cyan);
	
	FVector B;
	GetNodeLocation_Hie(1, HieOctree.Layers[0][0].MortonCode >> 3, B);
	DebugDrawVoxel(B, FVector(200), FColor::White);*/

	/*for (int32 i = 0; i < HieOctree.Layers[1].Num(); i ++)
	{
	FSVONavNode& Node1 = HieOctree.Layers[1][i];
	FVector NodeLocation1;
	GetNodeLocation_Hie(1, Node1.MortonCode, NodeLocation1);
	DebugDrawVoxel(NodeLocation1, FVector(200), FColor::Black);
	}*/

	/*for (int32 i = 0; i < HieOctree.Layers[2].Num(); i ++)
	{
	FSVONavNode& Node1 = HieOctree.Layers[2][i];
	FVector NodeLocation1;
	GetNodeLocation_Hie(2, Node1.MortonCode, NodeLocation1);
	DebugDrawVoxel(NodeLocation1, FVector(400), FColor::Black);
	}*/
}

void ASVONavVolume::Initialise()
{
	Octree.Reset();
	BlockedIndices.Empty();
	BlockedIndicesHie.Empty();
	HierarchyStartIndex.Empty();
	NumBytes = 0;

	HieOctree.Reset();

#if WITH_EDITOR
	FlushDebugDraw();
#endif

	UpdateVolume();
}

void ASVONavVolume::InitRasterize()
{
	BlockedIndices.Emplace();
	BlockedIndicesHie.Emplace();

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
				if (IsBlocked(NodeLocation, VoxelHalfSizes[0]))
				{
					RasterizeLeaf(NodeLocation, LeafIndex);
					NewNode.FirstChild.SetLayerIndex(0);
					NewNode.FirstChild.SetNodeIndex(LeafIndex);
					NewNode.FirstChild.SetSubNodeIndex(0);
					LeafIndex++;
				}
				else
				{
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
	for (int32 I = 0; I < 64; I++)
	{
		uint_fast32_t X, Y, Z;
		morton3D_64_decode(I, X, Y, Z);
		const FVector VoxelLocation = Location + FVector(X * VoxelScale, Y * VoxelScale, Z * VoxelScale) + VoxelScale *
			0.5f;
		if (LeafIndex >= Octree.Leaves.Num() - 1) Octree.Leaves.AddDefaulted(1);

		if (IsBlocked(VoxelLocation, VoxelScale * 0.5f))
		{
			Octree.Leaves[LeafIndex].SetSubNode(I);
		}
		else
		{
			//create first layer of hierarchical octree
			int32 Index = HieOctree.Layers[0].Emplace();
			FSVONavNode& NewNode = HieOctree.Layers[0][Index];

			FIntVector Voxel;
			GetMortonVoxel_Hie(VoxelLocation, 0, Voxel);
			const uint_fast64_t MortonCode = morton3D_64_encode(Voxel.X, Voxel.Y, Voxel.Z);
			NewNode.MortonCode = MortonCode;
		}
	}
	/*for (int32 I = 0; I < 8; I++)
	{
		uint_fast32_t X, Y, Z;
		morton3D_64_decode(I, X, Y, Z);
		const float Scale = VoxelHalfSizes_Hie[1] * 2;
		const FVector VoxelLocation = Location + FVector(X * Scale, Y * Scale, Z * Scale) + VoxelHalfSizes_Hie[1];


		//create second layer of hierarchical octree
		int32 Index = HieOctree.Layers[1].Emplace();
		FSVONavNode& NewNode = HieOctree.Layers[1][Index];

		FIntVector Voxel;
		GetMortonVoxel_Hie(VoxelLocation, 1, Voxel);
		const uint_fast64_t MortonCode = morton3D_64_encode(Voxel.X, Voxel.Y, Voxel.Z);
		NewNode.MortonCode = MortonCode;
	}*/
}

void ASVONavVolume::BuildLinks(uint8 LayerIndex)
{
	if (Octree.Layers.Num() == 0) return;
	TArray<FSVONavNode>& layer = GetLayer(LayerIndex);
	layerindex_t searchLayer = LayerIndex;

	// For each node
	for (nodeindex_t i = 0; i < layer.Num(); i++)
	{
		FSVONavNode& node = layer[i];

		// Get our world co-ordinate
		uint_fast32_t x, y, z;
		morton3D_64_decode(node.MortonCode, x, y, z);
		nodeindex_t backtrackIndex = -1;
		nodeindex_t index = i;
		FVector nodePos;
		GetNodeLocation(LayerIndex, node.MortonCode, nodePos);

		// For each direction
		for (int d = 0; d < 6; d++)
		{
			FSVONavLink& linkToUpdate = node.Neighbours[d];

			backtrackIndex = index;

			while (!FindLink(searchLayer, index, d, linkToUpdate, nodePos) && LayerIndex < Octree.Layers.Num() - 2)
			{
				FSVONavLink& parent = GetLayer(searchLayer)[index].Parent;
				if (parent.IsValid())
				{
					index = parent.NodeIndex;
					searchLayer = parent.LayerIndex;
				}
				else
				{
					searchLayer++;
					GetIndexForCode(searchLayer, node.MortonCode >> 3, index);
				}
			}
			index = backtrackIndex;
			searchLayer = LayerIndex;
		}
	}
}

void ASVONavVolume::BuildHieOctree()
{
	RasterizeFirstLayer_Hie();
	for (int32 i = 2; i < NumLayer_Hie; i++)
	{
		RasterizSparseLayer_Hie(i);
	}
	for (int32 i = 0; i < NumLayer_Hie; i ++)
	{
		HierarchyStartIndex.Add(HieOctree.Layers[i].Num() - 1);
	}
	//RasterizSparseLayer_Hie(2);
	//RasterizSparseLayer_Hie(3);
	//RasterizSparseLayer_Hie(2);
	//RasterizSparseLayer_Hie(3);
	BuildLinks_Hie(0);
	for (int32 i = 1; i < NumLayer_Hie; i++)
	{
		BuildHierarchyNodes_Hie(i);
		BuildSecondsLinks_Hie(i);
	}
	/*for (int32 i = 0; i < NumLayer_Hie; i++)
	{
		if (i == 0)
		{
			BuildLinks_Hie(0);
		}
		if (i == 1)
		{
			RasterizeFirstLayer_Hie();
			BuildSecondsLinks_Hie(i);
		}
		if (i > 1)
		{
			RasterizeLayer_Hie(i);
			BuildSecondsLinks_Hie(i);
		}
	}*/
}

void ASVONavVolume::RasterizSparseLayer_Hie(layerindex_t LayerIndex)
{
	HieOctree.Layers.Emplace();
	HieOctree.Layers[LayerIndex].Reserve(BlockedIndices[LayerIndex - 2].Num() * 8);
	const int32 NumNodes = GetLayerNodeCount_Hie(LayerIndex);
	for (int32 I = 0; I < NumNodes; I++)
	{
		if (BlockedIndices[LayerIndex - 2].Contains(I >> 3))
		{
			const int32 Index = HieOctree.Layers[LayerIndex].Emplace();
			FSVONavNode& NewNode = HieOctree.Layers[LayerIndex][Index];
			NewNode.MortonCode = I;
		}
	}
}

void ASVONavVolume::BuildSecondsLinks_Hie(layerindex_t LayerIndex)
{
	if (HieOctree.Layers.Num() == 0) return;
	TArray<FSVONavNode>& LayerNodes = HieOctree.Layers[LayerIndex];

	for (int32 I = 0; I < LayerNodes.Num(); I++)
	{
		//if(LayerIndex == 1) UE_LOG(LogTemp, Warning, TEXT("ChildNum: %i"), LayerNodes[I].GetChildNum());
		FSVONavNode& Node = LayerNodes[I];

		if (Node.HasChildren())
		{
			TArray<FSVONavLink> NeighbourLinks;
			for (int32 C = 0; C < Node.Childs.Num(); C ++)
			{
				if (!Node.Childs[C].IsValid()) continue;
				const FSVONavNode& Child = GetNode_Hie(Node.Childs[C]);
				for (int32 A = 0; A < 12; A ++)
				{
					const FSVONavLink& ChildNeighbourLink = Child.Neighbours[A];
					if (ChildNeighbourLink.IsValid())
					{
						const FSVONavNode& ChildNeighbourNode = GetNode_Hie(ChildNeighbourLink);
						FSVONavLink InitLink;
						if (ChildNeighbourLink.GetLayerIndex() >= LayerIndex && ChildNeighbourLink.GetLayerIndex() < NumLayer_Hie -1)
						{
							InitLink.SetNodeIndex(ChildNeighbourLink.GetNodeIndex());
							InitLink.SetLayerIndex(ChildNeighbourLink.GetLayerIndex());
						}
						else if (ChildNeighbourLink.GetLayerIndex() < LayerIndex)
						{
							if(ChildNeighbourNode.Parent.LayerIndex != LayerIndex || ChildNeighbourNode.Parent.NodeIndex != I)
							{
								InitLink.SetNodeIndex(ChildNeighbourNode.Parent.NodeIndex);
								InitLink.SetLayerIndex(ChildNeighbourNode.Parent.LayerIndex);
							}
						}
						//ConnectedNeighbourIndex.AddUnique(ChildNeighbourNode.Parent.NodeIndex);
						if (!NeighbourLinks.Contains(InitLink) && InitLink.IsValid())
						{
							NeighbourLinks.AddUnique(InitLink);
						}
						
					}
				}
			}
			for (int32 V = 0; V < 12; V++)
			{
				if (V <= NeighbourLinks.Num() - 1)
				{
					Node.Neighbours[V].SetNodeIndex(NeighbourLinks[V].GetNodeIndex());
					Node.Neighbours[V].SetLayerIndex(NeighbourLinks[V].GetLayerIndex());
#if WITH_EDITOR
					if (LayerIndex != 0)
					{
						FVector NodeLocation;
						GetNodeLocation_Hie(LayerIndex, Node.MortonCode, NodeLocation);

						FVector NeighbourLocation;
						GetNodeLocation_Hie(Node.Neighbours[V].GetLayerIndex(),
                                            GetNode_Hie(Node.Neighbours[V]).MortonCode, NeighbourLocation);
						DebugLinks.Add(FSVONavDebugLink(NodeLocation, NeighbourLocation, LayerIndex));
					}
#endif
					continue;
				}
				Node.Neighbours[V].Invalidate();
			}
		}
		else
		{
			FVector NodeLocation;
			GetNodeLocation_Hie(LayerIndex, Node.MortonCode, NodeLocation);
			if (!IsBlocked(NodeLocation, VoxelHalfSizes_Hie[LayerIndex]))
			{
				for (int32 Direction = 0; Direction < 6; Direction++)
				{
					//int32 NodeIndex = I;
					mortoncode_t CurrentCode = Node.MortonCode;
					mortoncode_t OriginalCode = CurrentCode;
					FSVONavLink& Link = Node.Neighbours[Direction];
					uint8 CurrentLayer = LayerIndex;
					//FindLinkViaCode_Hie(CurrentLayer, CurrentCode, Direction, Link, NodeLocation);
					while (!FindLinkViaCodeChildlessNode_Hie(CurrentLayer, CurrentCode, OriginalCode, Direction, Link,
					                                         NodeLocation) &&
						CurrentLayer < HieOctree.Layers.Num() - 2)
					{
						CurrentLayer++;
						CurrentCode = CurrentCode >> 3;
					}
					//Node.Neighbours[Direction + 6].Invalidate();
				}
			}
		}
		/*if (LayerIndex == 2)
		{
			UE_LOG(LogTemp, Warning, TEXT("%i has %i neighbour"), I, ConnectedNeighbourIndex.Num());
			for (int32 V = 0; V < ConnectedNeighbourIndex.Num(); V++)
			{
				UE_LOG(LogTemp, Warning, TEXT("From num: %i to %i"), I, ConnectedNeighbourIndex[V]);
			}
		}*/

		/*for (int32 Direction = 0; Direction < 6; Direction++)
		{
			int32 NodeIndex = I;
			FSVONavLink& Link = Node.Neighbours[Direction];
			uint8 CurrentLayer = LayerIndex;
			CombineChildLink_Hie(CurrentLayer, NodeIndex, Direction, Link, NodeLocation);
		}*/
	}
}

bool ASVONavVolume::CombineChildLink_Hie(layerindex_t LayerIndex, int32 NodeIndex, uint8 Direction, FSVONavLink& Link,
                                         const FVector& NodeLocation)
{
	TArray<FSVONavNode>& LayerNodes = HieOctree.Layers[LayerIndex];
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

	TArray<int32> NeighbourNodeIndexes = GetArrayNodeIndex_Hie(LayerIndex, AdjacentCode);
	if (NeighbourNodeIndexes.Num() > 0)
	{
		TArray<int32> ConnectedNeighbourIndex;

		for (int32 I = 0; I < 8; I++)
		{
			for (int32 A = 0; A < 12; A ++)
			{
				int32 ChildNeighbourIndex = HieOctree.Layers[LayerIndex - 1][TargetNode.Childs[I].NodeIndex].Neighbours[
					A].NodeIndex;
				if (HieOctree.Layers[LayerIndex - 1][TargetNode.Childs[I].NodeIndex].Neighbours[A].IsValid() &&
					HieOctree.Layers[LayerIndex - 1][TargetNode.Childs[I].NodeIndex].Parent.NodeIndex != NodeIndex)
				{
					UE_LOG(LogTemp, Warning, TEXT("num: %i"), ChildNeighbourIndex);
					ConnectedNeighbourIndex.Add(ChildNeighbourIndex);
				}
			}
		}

		if (NodeIndex == 0)
		{
			UE_LOG(LogTemp, Warning, TEXT("Neighbor num: %i"), ConnectedNeighbourIndex.Num());
			for (int32 I = 0; I < ConnectedNeighbourIndex.Num(); I++)
			{
				UE_LOG(LogTemp, Warning, TEXT("num: %i"), ConnectedNeighbourIndex[I]);
			}
		}
		/*Link.SetLayerIndex(LayerIndex);
		Link.SetNodeIndex(NeighbourNodeIndex);

#if WITH_EDITOR
		FVector NeighbourLocation;
		GetNodeLocation_Hie(LayerIndex, LayerNodes[NeighbourNodeIndex].MortonCode, NeighbourLocation);
		DebugLinks.Add(FSVONavDebugLink(NodeLocation, NeighbourLocation, LayerIndex));
#endif*/

		return true;
	}
	Link.Invalidate();
	return false;
}

TArray<int32> ASVONavVolume::GetArrayNodeIndex_Hie(layerindex_t LayerIndex, uint_fast64_t NodeMortonCode)
{
	TArray<int32> NodeSearchResults;
	const auto& OctreeLayer = HieOctree.Layers[LayerIndex];

	int32 Start = 0;
	int32 End = HierarchyStartIndex[LayerIndex];
	int32 Mean = (Start + End) * 0.5f;

	// Binary search by Morton code
	while (Start <= End)
	{
		if (OctreeLayer[Mean].MortonCode < NodeMortonCode) Start = Mean + 1;
		else if (OctreeLayer[Mean].MortonCode == NodeMortonCode)
		{
			NodeSearchResults.Add(Mean);
			NodeSearchResults.Append(GetArrayNodeIndex_HieExtra(LayerIndex, NodeMortonCode));
			return NodeSearchResults;
		}
		else End = Mean - 1;
		Mean = (Start + End) * 0.5f;
	}
	return NodeSearchResults;
}

TArray<int32> ASVONavVolume::GetArrayNodeIndex_HieExtra(layerindex_t LayerIndex, uint_fast64_t NodeMortonCode)
{
	TArray<int32> NodeSearchResults;
	const auto& OctreeLayer = HieOctree.Layers[LayerIndex];

	if (HierarchyStartIndex[LayerIndex] + 1 < OctreeLayer.Num())
	{
		int32 Start = HierarchyStartIndex[LayerIndex] + 1;
		int32 End = OctreeLayer.Num() - 1;
		int32 Mean = (Start + End) * 0.5f;
		int32 LayerDupOffset = 4 ^ LayerIndex - 1;
		int32 C = 0;
		while (Start <= End)
		{
			/*C++;
			check(C<100);*/
			if (OctreeLayer[Mean].MortonCode == NodeMortonCode)
			{
				NodeSearchResults.Add(Mean);

				for (int32 I = 1; I <= LayerDupOffset; I++)
				{
					bool HasDuplicateCode = false;
					if (Mean + I < OctreeLayer.Num() && OctreeLayer[Mean + I].MortonCode == NodeMortonCode)
					{
						HasDuplicateCode = true;
						NodeSearchResults.Add(Mean + I);
					}
					if (Mean - I >= HierarchyStartIndex[LayerIndex] + 1 && OctreeLayer[Mean - I].MortonCode ==
						NodeMortonCode)
					{
						HasDuplicateCode = true;
						NodeSearchResults.Add(Mean - I);
					}
					if (HasDuplicateCode == false)
					{
						break;
					}
				}
				return NodeSearchResults;
			}
			if (OctreeLayer[Mean].MortonCode < NodeMortonCode) Start = Mean + 1;
			else End = Mean - 1;
			Mean = (Start + End) * 0.5f;
		}
	}
	return NodeSearchResults;
}

void ASVONavVolume::RasterizeFirstLayer_Hie()
{
	HieOctree.Layers.Emplace();
	TArray<mortoncode_t> UniqueParentCodes;
	for (int32 I = 0; I < HieOctree.Layers[0].Num(); I++)
	{
		FSVONavNode& Child = HieOctree.Layers[0][I];
		mortoncode_t ParentCode = Child.MortonCode >> 3;
		if (!UniqueParentCodes.Contains(ParentCode))
		{
			UniqueParentCodes.Add(ParentCode);

			uint32 ParentIndex = HieOctree.Layers[1].Emplace();
			FSVONavNode& Parent = HieOctree.Layers[1][ParentIndex];
			Parent.MortonCode = ParentCode;
		}
	}
}

void ASVONavVolume::RasterizeLayer_Hie(layerindex_t Layer)
{
	HieOctree.Layers.Emplace();
	TArray<mortoncode_t> UniqueParentCodes;
	layerindex_t ChildLayer = Layer - 1;
	for (int32 I = 0; I < HieOctree.Layers[ChildLayer].Num(); I++)
	{
		FSVONavNode& Child = HieOctree.Layers[ChildLayer][I];
		mortoncode_t ParentCode = Child.MortonCode >> 3;
		mortoncode_t CacheCode = ParentCode;
		if (!UniqueParentCodes.Contains(ParentCode))
		{
			UniqueParentCodes.Add(ParentCode);

			TArray<mortoncode_t> ChildCodes;
			mortoncode_t FirstChildCode = CacheCode << 3;
			ChildCodes.Add(FirstChildCode);

			uint_fast32_t X, Y, Z;
			morton3D_64_decode(FirstChildCode, X, Y, Z);
			for (int32 C = 0; C < 7; C++)
			{
				int32 sX = X, sY = Y, sZ = Z;
				// Add the direction
				sX += FirstChildOffsetDir[C].X;
				sY += FirstChildOffsetDir[C].Y;
				sZ += FirstChildOffsetDir[C].Z;

				mortoncode_t NewChildCode = morton3D_64_encode(sX, sY, sZ);
				ChildCodes.Add(NewChildCode);
			}

			TArray<int32> ChildIndexes;
			for (int32 A = 0; A < 8; A++)
			{
				TArray<int32> PotentialChildIndexes = GetArrayNodeIndex_Hie(ChildLayer, ChildCodes[A]);

				if (PotentialChildIndexes.Num() > 0)
				{
					for (int32 V = 0; V < PotentialChildIndexes.Num(); V++)
						HieOctree.Layers[ChildLayer][PotentialChildIndexes[V]].Parent.SetLayerIndex(Layer);
					ChildIndexes.Append(PotentialChildIndexes);
				}
			}

			//child connection matrix
			TArray<TArray<int32>> ChildGroups;
			TArray<TArray<int32>> Region;
			for (int32 B = 0; B < ChildIndexes.Num(); B++)
			{
				ChildGroups.Emplace();

				for (int32 C = 0; C < 12; C++)
				{
					int32 CurrentNeighbourIndex = HieOctree.Layers[ChildLayer][ChildIndexes[B]].Neighbours[C].NodeIndex;
					if (HieOctree.Layers[ChildLayer][ChildIndexes[B]].Neighbours[C].IsValid())
					{
						if (ChildIndexes.Contains(CurrentNeighbourIndex))
						{
							ChildGroups[B].AddUnique(CurrentNeighbourIndex);

							if (Region.Num() < 1)
							{
								Region.Emplace();
								Region[0].AddUnique(CurrentNeighbourIndex);
								Region[0].AddUnique(ChildIndexes[B]);
							}
							else
							{
								bool RegionExisted = false;
								TArray<int32> RegionContainIndex;
								for (int32 D = 0; D < Region.Num(); D++)
								{
									if (Region[D].Contains(CurrentNeighbourIndex) || Region[D].Contains(ChildIndexes[B])
									)
									{
										RegionContainIndex.AddUnique(D);
									}
								}

								if (RegionContainIndex.Num() < 1)
								{
									int32 RegionIndex = Region.Emplace();
									Region[RegionIndex].AddUnique(CurrentNeighbourIndex);
									Region[RegionIndex].AddUnique(ChildIndexes[B]);
								}
								else if (RegionContainIndex.Num() == 1)
								{
									Region[RegionContainIndex[0]].AddUnique(CurrentNeighbourIndex);
									Region[RegionContainIndex[0]].AddUnique(ChildIndexes[B]);
								}
								else
								{
									for (int32 G = RegionContainIndex.Num() - 1; G > 0; G--)
									{
										for (int32 F = 0; F < Region[G].Num(); F++)
										{
											//Region[RegionContainIndex[0]].AddUnique(Region[RegionContainIndex[G]][F]);
											int32 RCI = RegionContainIndex[0];
											int32 RCI2 = RegionContainIndex[G];
											TArray<int32>& ar = Region[RCI2];
											int32 R2 = ar[F];
											Region[RCI].AddUnique(R2);
										}
										/*for (int32 F = Region[G].Num() - 1; F > 0; F--)
										{
											//Region[RegionContainIndex[0]].AddUnique(Region[RegionContainIndex[G]][F]);
											int32 RCI = RegionContainIndex[0];
											int32 RCI2 = RegionContainIndex[G];
											TArray<int32>& ar = Region[RCI2];
											int32 R2 = ar[F];
											Region[RCI].AddUnique(R2);
										}
										Region.RemoveAt(RegionContainIndex[G]);*/
									}
								}
							}
						}
						if (ChildGroups[B].Num() > 3) break;
					}
				}
			}

			for (int32 F = 0; F < ChildGroups.Num(); F++)
			{
				if (ChildGroups[F].Num() < 1)
				{
					int32 RegionIndex = Region.Emplace();
					Region[RegionIndex].Add(ChildIndexes[F]);
				}
			}

			for (int32 J = 0; J < Region.Num(); J++)
			{
				uint32 ParentIndex = HieOctree.Layers[Layer].Emplace();
				FSVONavNode& Parent = HieOctree.Layers[Layer][ParentIndex];
				Parent.MortonCode = ParentCode;
				Parent.FirstChild.SetLayerIndex(ChildLayer);
				Parent.FirstChild.SetNodeIndex(Region[J][0]);

				int32 ValidChildNum = Region[J].Num();
				for (int32 M = 0; M < ValidChildNum; M++)
				{
					HieOctree.Layers[ChildLayer][Region[J][M]].Parent.SetNodeIndex(ParentIndex);

					Parent.Childs[M].SetLayerIndex(ChildLayer);
					Parent.Childs[M].SetNodeIndex(Region[J][M]);
				}
				for (int32 P = ValidChildNum; P < 8; P++)
				{
					Parent.Childs[P].Invalidate();
				}
			}
		}
	}
}

void ASVONavVolume::BuildHierarchyNodes_Hie(layerindex_t Layer)
{
	layerindex_t ChildLayer = Layer - 1;
	TArray<mortoncode_t> UniqueParentCodes;

	for (int32 I = 0; I < HieOctree.Layers[ChildLayer].Num(); I++)
	{
		FSVONavNode& Child = HieOctree.Layers[ChildLayer][I];
		mortoncode_t ParentCode = Child.MortonCode >> 3;
		mortoncode_t CacheCode = ParentCode;
		if (!UniqueParentCodes.Contains(ParentCode))
		{
			UniqueParentCodes.Add(ParentCode);

			TArray<mortoncode_t> ChildCodes;
			mortoncode_t FirstChildCode = CacheCode  << 3;
			ChildCodes.Add(FirstChildCode);

			uint_fast32_t X, Y, Z;
			morton3D_64_decode(FirstChildCode, X, Y, Z);
			for (int32 C = 0; C < 7; C++)
			{
				int32 sX = X, sY = Y, sZ = Z;
				// Add the direction
				sX += FirstChildOffsetDir[C].X;
				sY += FirstChildOffsetDir[C].Y;
				sZ += FirstChildOffsetDir[C].Z;

				mortoncode_t NewChildCode = morton3D_64_encode(sX, sY, sZ);
				ChildCodes.Add(NewChildCode);
			}

			TArray<int32> ChildIndexes;
			for (int32 A = 0; A < 8; A++) ChildIndexes.Append(GetArrayNodeIndex_Hie(ChildLayer, ChildCodes[A]));

			for (int32 V = 0; V < ChildIndexes.Num(); V++)
				HieOctree.Layers[ChildLayer][ChildIndexes[V]].Parent.SetLayerIndex(Layer);
			
			TArray<TArray<int32>> ChildGroups;
			for (int32 B = 0; B < ChildIndexes.Num(); B++)
			{
				int32 ChildIndex = ChildGroups.Emplace();
				ChildGroups[ChildIndex].Add(ChildIndexes[B]);
				for (int32 C = 0; C < 12; C++)
				{
					FSVONavLink& NeighbourLink = HieOctree.Layers[ChildLayer][ChildIndexes[B]].Neighbours[C];
					int32 CurrentNeighbourIndex = NeighbourLink.NodeIndex;
					if (NeighbourLink.GetLayerIndex() == ChildLayer && ChildIndexes.Contains(CurrentNeighbourIndex))
					{
						ChildGroups[B].AddUnique(CurrentNeighbourIndex);
					}
				}
			}

			TArray<TArray<int32>> Region = ChildGroups;
			int32 C = Region.Num() - 1;
			while (C > 0)
			{
				bool ConnectionFound = false;
				for (int32 D = 0; D < Region[C].Num(); D++)
				{
					for (int32 E = 0; E < Region.Num(); E++)
					{
						if (E == C) break;
						if (Region[E].Contains(Region[C][D]))
						{
							for (int32 F = 0; F < Region[C].Num(); F++)
							{
								Region[E].AddUnique(Region[C][F]);
							}
							Region.RemoveAt(C, 1, true);
							ConnectionFound = true;
							break;
						}
					}
					if (ConnectionFound) break;
				}
				if (!ConnectionFound) C--;
				else C = Region.Num() - 1;
			}

			for (int32 J = 0; J < Region.Num(); J++)
			{
				int32 ParentIndex = 0;
				if (J == 0)
				{
					if(!GetNodeIndex_Hie(Layer, ParentCode, ParentIndex))
					{
						ParentIndex = HieOctree.Layers[Layer].Emplace();
					}
				}
				else
				{
					ParentIndex = HieOctree.Layers[Layer].Emplace();
				}
				FSVONavNode& Parent = HieOctree.Layers[Layer][ParentIndex];
				Parent.MortonCode = ParentCode;

				Parent.FirstChild.SetLayerIndex(ChildLayer);
				Parent.FirstChild.SetNodeIndex(Region[J][0]);
				int32 ValidChildNum = Region[J].Num();
				/*if(ValidChildNum  <= 8)
				{
					for(int32 Y = 0; Y < ChildGroups.Num(); Y++)
					{
						
					}
				}*/
				
				for (int32 M = 0; M < ValidChildNum; M++)
				{
					HieOctree.Layers[ChildLayer][Region[J][M]].Parent.SetNodeIndex(ParentIndex);
					
					//Parent.Childs[M].SetLayerIndex(ChildLayer);
					//Parent.Childs[M].SetNodeIndex(Region[J][M]);
					Parent.Childs.Add(FSVONavLink(ChildLayer, Region[J][M],0));
				}
				/*for (int32 P = ValidChildNum; P < 8; P++)
				{
					Parent.Childs[P].Invalidate();
				}*/
			}
		}
	}
}

void ASVONavVolume::GetMortonVoxel_Hie(const FVector& Location, int32 LayerIndex, FIntVector& MortonLocation) const
{
	const FVector LocationLocal = Location - (VolumeOrigin - VolumeExtent);
	const float Size = VoxelHalfSizes_Hie[LayerIndex] * 2;
	MortonLocation.X = FMath::FloorToInt(LocationLocal.X / Size);
	MortonLocation.Y = FMath::FloorToInt(LocationLocal.Y / Size);
	MortonLocation.Z = FMath::FloorToInt(LocationLocal.Z / Size);
}

void ASVONavVolume::BuildLinks_Hie(layerindex_t LayerIndex)
{
	if (HieOctree.Layers.Num() == 0) return;
	TArray<FSVONavNode>& LayerNodes = HieOctree.Layers[LayerIndex];

	for (int32 I = 0; I < LayerNodes.Num(); I++)
	{
		FSVONavNode& Node = LayerNodes[I];

		FVector NodeLocation;
		GetNodeLocation_Hie(LayerIndex, Node.MortonCode, NodeLocation);

		for (int32 Direction = 0; Direction < 6; Direction++)
		{
			//int32 NodeIndex = I;
			mortoncode_t CurrentCode = Node.MortonCode;
			mortoncode_t OriginalCode = CurrentCode;
			FSVONavLink& Link = Node.Neighbours[Direction];
			uint8 CurrentLayer = LayerIndex;
			//FindLinkViaCode_Hie(CurrentLayer, CurrentCode, Direction, Link, NodeLocation);
			while (!FindLinkViaCode_Hie(CurrentLayer, CurrentCode, OriginalCode, Direction, Link, NodeLocation) &&
				CurrentLayer < HieOctree.Layers.Num() - 2)
			{
				CurrentLayer++;
				CurrentCode = CurrentCode >> 3;
			}
			//Node.Neighbours[Direction + 6].Invalidate();
		}
	}
}

bool ASVONavVolume::FindLink_Hie(layerindex_t LayerIndex, int32 NodeIndex, uint8 Direction, FSVONavLink& Link,
                                 const FVector& NodeLocation)
{
	TArray<FSVONavNode>& LayerNodes = HieOctree.Layers[LayerIndex];
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

bool ASVONavVolume::FindLinkViaCodeChildlessNode_Hie(layerindex_t LayerIndex, mortoncode_t MortonCode,
                                                     mortoncode_t OriginalCode, uint8 Direction, FSVONavLink& Link,
                                                     const FVector& NodeLocation)
{
	const int32 MaxCoordinate = GetSegmentNodeCount_Hie(LayerIndex);
	TArray<FSVONavNode>& LayerNodes = HieOctree.Layers[LayerIndex];

	uint_fast32_t X, Y, Z;
	morton3D_64_decode(MortonCode, X, Y, Z);

	uint_fast32_t OX, OY, OZ;
	morton3D_64_decode(OriginalCode, OX, OY, OZ);

	FIntVector S(static_cast<int32>(X), static_cast<int32>(Y), static_cast<int32>(Z));
	S += Directions[Direction];
	//do check range here
	if (S.X < 0 || S.X >= MaxCoordinate ||
		S.Y < 0 || S.Y >= MaxCoordinate ||
		S.Z < 0 || S.Z >= MaxCoordinate)
	{
		Link.Invalidate();
		return true;
	}

	X = S.X;
	Y = S.Y;
	Z = S.Z;

	const uint_fast64_t AdjacentCode = morton3D_64_encode(X, Y, Z);

	int32 NeighbourNodeIndex;
	if (GetNodeIndex_Hie(LayerIndex, AdjacentCode, NeighbourNodeIndex))
	{
		FVector Location;
		GetNodeLocation_Hie(LayerIndex, morton3D_64_encode(X, Y, Z), Location);
		if (IsBlocked(Location, VoxelHalfSizes_Hie[LayerIndex]))
		{
			Link.Invalidate();
			return true;
		}

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

bool ASVONavVolume::FindLinkViaCode_Hie(layerindex_t LayerIndex, mortoncode_t MortonCode, mortoncode_t OriginalCode,
                                        uint8 Direction,
                                        FSVONavLink& Link, const FVector& NodeLocation)
{
	const int32 MaxCoordinate = GetSegmentNodeCount_Hie(LayerIndex);
	TArray<FSVONavNode>& LayerNodes = HieOctree.Layers[LayerIndex];

	uint_fast32_t X, Y, Z;
	morton3D_64_decode(MortonCode, X, Y, Z);

	uint_fast32_t OX, OY, OZ;
	morton3D_64_decode(OriginalCode, OX, OY, OZ);

	FIntVector S(static_cast<int32>(X), static_cast<int32>(Y), static_cast<int32>(Z));
	S += Directions[Direction];
	//do check range here
	if (S.X < 0 || S.X >= MaxCoordinate ||
		S.Y < 0 || S.Y >= MaxCoordinate ||
		S.Z < 0 || S.Z >= MaxCoordinate)
	{
		Link.Invalidate();
		return true;
	}

	X = S.X;
	Y = S.Y;
	Z = S.Z;

	const uint_fast64_t AdjacentCode = morton3D_64_encode(X, Y, Z);

	int32 NeighbourNodeIndex;
	if (GetNodeIndex_Hie(LayerIndex, AdjacentCode, NeighbourNodeIndex))
	{
		if (LayerIndex != 0)
		{
			FVector Location;
			GetNodeLocation_Hie(LayerIndex, morton3D_64_encode(X, Y, Z), Location);
			if (IsBlocked(Location, VoxelHalfSizes_Hie[LayerIndex]))
			{
				Link.Invalidate();
				return true;
			}
		}

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

	if (Direction == 0 && (OX + 5) % 4 != 0) return true;
	if (Direction == 1 && (OX + 1) % 4 != 1) return true;
	if (Direction == 2 && (OY + 5) % 4 != 0) return true;
	if (Direction == 3 && (OY + 1) % 4 != 1) return true;
	if (Direction == 4 && (OZ + 5) % 4 != 0) return true;
	if (Direction == 5 && (OZ + 1) % 4 != 1) return true;
	return false;
}

int32 ASVONavVolume::GetSegmentNodeCount_Hie(layerindex_t LayerIndex) const
{
	return FMath::Pow(2, VoxelExponent_Hie - LayerIndex);
}

bool ASVONavVolume::GetNodeIndex_Hie(layerindex_t LayerIndex, uint_fast64_t NodeMortonCode, int32& NodeIndex) const
{
	const auto& OctreeLayer = HieOctree.Layers[LayerIndex];
	int32 Start = 0;
	int32 End = HierarchyStartIndex.Num() == 0 ? OctreeLayer.Num() - 1 : HierarchyStartIndex[LayerIndex];
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

float ASVONavVolume::GetVoxelScale_Hie(uint8 LayerIndex) const
{
	return VolumeExtent.X / FMath::Pow(2.0f, VoxelExponent_Hie) * FMath::Pow(2.0f, LayerIndex + 1);
}

bool ASVONavVolume::GetNodeLocation_Hie(uint8 LayerIndex, uint_fast64_t MortonCode, FVector& Location) const
{
	const float Scale = VoxelHalfSizes_Hie[LayerIndex] * 2;
	uint_fast32_t X, Y, Z;
	morton3D_64_decode(MortonCode, X, Y, Z);
	Location = VolumeOrigin - VolumeExtent + Scale * FVector(X, Y, Z) + FVector(Scale * 0.5f);
	return true;
}

void ASVONavVolume::GetNeighbourLinks_Hie(const FSVONavLink& Link, TArray<FSVONavLink>& NeighbourLinks) const
{
}

int32 ASVONavVolume::GetLayerNodeCount_Hie(layerindex_t LayerIndex) const
{
	return FMath::Pow(8, VoxelExponent_Hie - LayerIndex);
}

const FSVONavNode& ASVONavVolume::GetNode_Hie(const FSVONavLink& Link) const
{
	return HieOctree.Layers[Link.LayerIndex][Link.NodeIndex];
}

FColor ASVONavVolume::GetLayerColour_Hie(const int32 LayerIndex) const
{
	const float Ratio = LayerColours.Num() / static_cast<float>(NumLayer_Hie) * LayerIndex;
	const int32 FirstIndex = FMath::FloorToInt(Ratio);
	const int32 LastIndex = FMath::Min(FMath::CeilToInt(Ratio), LayerColours.Num() - 1);
	const float Lerp = FMath::Fmod(Ratio, 1);
	return FColor(
		FMath::Lerp(LayerColours[FirstIndex].R, LayerColours[LastIndex].R, Lerp),
		FMath::Lerp(LayerColours[FirstIndex].G, LayerColours[LastIndex].G, Lerp),
		FMath::Lerp(LayerColours[FirstIndex].B, LayerColours[LastIndex].B, Lerp)
	);
}

bool ASVONavVolume::GetLink_Hie(const FVector& Location, FSVONavLink& Link)
{
	if (!IsWithinBounds(Location)) return false;

	int32 LayerIndex = HieOctree.Layers.Num() - 2;
	while (LayerIndex >= 0)
	{
		const TArray<FSVONavNode>& Layer = GetLayer_Hie(LayerIndex);
		FIntVector Voxel;
		GetMortonVoxel_Hie(Location, LayerIndex, Voxel);
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

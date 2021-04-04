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

	if (bDisplayNeighbourLink)
	{
		DebugLinks.Empty();
		//BuildSecondsLinks_Hie(1);
		BuildLinks_Hie(0);
		UE_LOG(LogTemp, Warning, TEXT("Debug Num: %i"), DebugLinks.Num());
		DebugDrawNeighbourLink();
	}

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

	UE_LOG(LogTemp, Warning, TEXT("%i"), GetLayerNodeCount(1));

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
	for (int32 i = 0; i < NumLayer_Hie; i++)
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
	}
}

void ASVONavVolume::BuildSecondsLinks_Hie(layerindex_t LayerIndex)
{
	if (HieOctree.Layers.Num() == 0) return;
	TArray<FSVONavNode>& LayerNodes = HieOctree.Layers[LayerIndex];

	for (int32 I = 0; I < LayerNodes.Num(); I++)
	{
		FSVONavNode& Node = LayerNodes[I];

		TArray<int32> ConnectedNeighbourIndex;

		for (int32 C = 0; C < 8; C ++)
		{
			if (!Node.Childs[C].IsValid()) continue;
			const FSVONavNode& Child = GetNode_Hie(Node.Childs[C]);
			for (int32 A = 0; A < 12; A ++)
			{
				const FSVONavLink& ChildNeighbourLink = Child.Neighbours[A];
				if (ChildNeighbourLink.IsValid())
				{
					const FSVONavNode& ChildNeighbourNode = GetNode_Hie(ChildNeighbourLink);
					if (ChildNeighbourNode.Parent.NodeIndex != I)
					{
						ConnectedNeighbourIndex.AddUnique(ChildNeighbourNode.Parent.NodeIndex);
					}
				}
			}
		}
		for (int32 V = 0; V < 12; V++)
		{
			if (V <= ConnectedNeighbourIndex.Num() - 1)
			{
				Node.Neighbours[V].SetLayerIndex(LayerIndex);
				Node.Neighbours[V].SetNodeIndex(ConnectedNeighbourIndex[V]);
#if WITH_EDITOR
				FVector NodeLocation;
				GetNodeLocation_Hie(LayerIndex, Node.MortonCode, NodeLocation);

				FVector NeighbourLocation;
				GetNodeLocation_Hie(LayerIndex, LayerNodes[ConnectedNeighbourIndex[V]].MortonCode, NeighbourLocation);
				DebugLinks.Add(FSVONavDebugLink(NodeLocation, NeighbourLocation, LayerIndex));
#endif
				continue;
			}
			Node.Neighbours[V].Invalidate();
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

	TArray<int32> NeighbourNodeIndexes;
	if (GetArrayNodeIndex_Hie(LayerIndex, AdjacentCode, NeighbourNodeIndexes))
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

bool ASVONavVolume::GetArrayNodeIndex_Hie(layerindex_t LayerIndex, uint_fast64_t NodeMortonCode,
                                          TArray<int32>& NodeIndexes) const
{
	const auto& OctreeLayer = HieOctree.Layers[LayerIndex];
	int32 Start = 0;
	int32 End = OctreeLayer.Num() - 1;
	int32 Mean = (Start + End) * 0.5f;
	// Binary search by Morton code
	while (Start <= End)
	{
		if (OctreeLayer[Mean].MortonCode < NodeMortonCode) Start = Mean + 1;
		else if (OctreeLayer[Mean].MortonCode == NodeMortonCode)
		{
			// layer maybe nodes with same morton code
			int32 LayerDupOffset = 4 ^ LayerIndex - 1;
			/*int32 StartIndex = FMath::Clamp(Mean - LayerDupOffset, 0, OctreeLayer.Num() - 1);
			int32 EndIndex = FMath::Clamp(Mean + LayerDupOffset, 0, OctreeLayer.Num() - 1);*/

			NodeIndexes.Add(Mean);

			for (int32 I = 1; I <= LayerDupOffset; I++)
			{
				bool HasDuplicateCode = false;
				if (Mean + I < OctreeLayer.Num() && OctreeLayer[Mean + I].MortonCode == NodeMortonCode)
				{
					HasDuplicateCode = true;
					NodeIndexes.Add(Mean + I);
				}
				if (Mean - I >= 0 && OctreeLayer[Mean - I].MortonCode == NodeMortonCode)
				{
					HasDuplicateCode = true;
					NodeIndexes.Insert(Mean - I, 0);
				}
				if (!HasDuplicateCode)
				{
					return true;
				}
			}
		}
		else End = Mean - 1;
		Mean = (Start + End) * 0.5f;
	}
	if (NodeIndexes.Num() > 0)
	{
		return true;
	}
	return false;
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

			int32 ChildIndex = 0;
			/*if (GetNodeIndex_Hie(0, Parent.MortonCode << 3, ChildIndex))
			{
				Parent.FirstChild.SetLayerIndex(0);
				Parent.FirstChild.SetNodeIndex(ChildIndex);

				for(int32 N = 0; N < 6; N ++)
				{
					const FSVONavLink& NeighbourLink = GetNode_Hie(Parent.FirstChild).Neighbours[N];
					if(NeighbourLink.IsValid())
					{
						const FSVONavNode& Neighbour = GetNode_Hie(NeighbourLink);
						if(Neighbour.MortonCode >> 3 == Parent.MortonCode)
						{
							Neighbour.Parent
						}
					}
				}

			
			}*/
			TArray<mortoncode_t> ChildCodes;
			mortoncode_t FirstChildCode = ParentCode << 3;
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
				int32 PotentialChildIndex = 0;

				if (GetNodeIndex_Hie(0, ChildCodes[A], PotentialChildIndex))
				{
					HieOctree.Layers[0][PotentialChildIndex].Parent.SetLayerIndex(1);
					ChildIndexes.Add(PotentialChildIndex);
				}
			}


			/*if(ChildIndexes.Num() > 5)
			{
				Parent.FirstChild.SetNodeIndex(ChildIndexes[0]);
				Parent.Child[0].SetLayerIndex(0);
				for(int32 T = 0; T < ChildIndexes.Num(); T++ )
				{
					Parent.Child[T].SetNodeIndex(ChildIndexes[T]);
				}
				UE_LOG(LogTemp, Warning, TEXT("Connected"));
				continue;
			}

			if(ChildIndexes.Num() <2)
			{
				Parent.FirstChild.SetNodeIndex(ChildIndexes[0]);
				Parent.Child[0].SetLayerIndex(0);
				Parent.Child[0].SetNodeIndex(ChildIndexes[0]);
				UE_LOG(LogTemp, Warning, TEXT("Connected"));
				continue;
			}

			if(ChildIndexes.Num() < 3)
			{
				for(int32 C = 0; C < 6; C++)
				{
					if(HieOctree.Layers[0][ChildIndexes[0]].Neighbours[C].IsValid())
					{
						if(ChildIndexes[1] == HieOctree.Layers[0][ChildIndexes[0]].Neighbours[C].NodeIndex)
						{
							Parent.FirstChild.SetNodeIndex(ChildIndexes[0]);
							UE_LOG(LogTemp, Warning, TEXT("Connected"));
							break;
							
						}
					}
				}
				continue;
			}*/

			/*if(ChildIndexes.Num() < 6)
			{*/
			//child connection matrix
			TArray<TArray<int32>> ChildGroups;
			TArray<TArray<int32>> Region;
			for (int32 B = 0; B < ChildIndexes.Num(); B++)
			{
				ChildGroups.Emplace();

				for (int32 C = 0; C < 6; C++)
				{
					int32 CurrentNeighbourIndex = HieOctree.Layers[0][ChildIndexes[B]].Neighbours[C].NodeIndex;
					if (HieOctree.Layers[0][ChildIndexes[B]].Neighbours[C].IsValid())
					{
						if (ChildIndexes.Contains(CurrentNeighbourIndex))
						{
							ChildGroups[B].Add(CurrentNeighbourIndex);

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
										RegionContainIndex.Add(D);
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
											Region[RegionContainIndex[0]].AddUnique(Region[RegionContainIndex[G]][F]);
										}
										Region.RemoveAt(RegionContainIndex[G]);
									}
								}
							}
						}
						if (ChildGroups[B].Num() > 3) break;
					}
				}
			}

			//TArray<int32> IsolatedChildGroup;

			for (int32 F = 0; F < ChildGroups.Num(); F++)
			{
				if (ChildGroups[F].Num() < 1)
				{
					int32 RegionIndex = Region.Emplace();
					Region[RegionIndex].Add(ChildIndexes[F]);
					//IsolatedChildGroup.Add(F);
				}
			}

			for (int32 J = 0; J < Region.Num(); J++)
			{
				uint32 ParentIndex = HieOctree.Layers[1].Emplace();
				FSVONavNode& Parent = HieOctree.Layers[1][ParentIndex];
				Parent.MortonCode = ParentCode;
				Parent.FirstChild.SetLayerIndex(0);
				Parent.FirstChild.SetNodeIndex(Region[J][0]);
				int32 ValidChildNum = Region[J].Num();
				for (int32 M = 0; M < ValidChildNum; M++)
				{
					HieOctree.Layers[0][Region[J][M]].Parent.SetNodeIndex(ParentIndex);
					Parent.Childs[M].SetLayerIndex(0);
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

void ASVONavVolume::RasterizeLayer_Hie(layerindex_t Layer)
{
	HieOctree.Layers.Emplace();
	TArray<mortoncode_t> UniqueParentCodes;
	layerindex_t ChildLayer = Layer - 1;
	for (int32 I = 0; I < HieOctree.Layers[ChildLayer].Num(); I++)
	{
		FSVONavNode& Child = HieOctree.Layers[ChildLayer][I];
		mortoncode_t ParentCode = Child.MortonCode >> 3;
		if (!UniqueParentCodes.Contains(ParentCode))
		{
			UniqueParentCodes.Add(ParentCode);

			TArray<mortoncode_t> ChildCodes;
			mortoncode_t FirstChildCode = ParentCode << 3;
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
				TArray<int32> PotentialChildIndexes;

				if (GetArrayNodeIndex_Hie(ChildLayer, ChildCodes[A], PotentialChildIndexes))
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
			int32 NodeIndex = I;
			FSVONavLink& Link = Node.Neighbours[Direction];
			uint8 CurrentLayer = LayerIndex;
			FindLink_Hie(CurrentLayer, NodeIndex, Direction, Link, NodeLocation);
			Node.Neighbours[Direction + 6].Invalidate();
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
		FVector NeighbourLocation;
		GetNodeLocation_Hie(LayerIndex, LayerNodes[NeighbourNodeIndex].MortonCode, NeighbourLocation);
		DebugLinks.Add(FSVONavDebugLink(NodeLocation, NeighbourLocation, LayerIndex));
#endif

		return true;
	}
	Link.Invalidate();
	return false;
}

bool ASVONavVolume::GetNodeIndex_Hie(layerindex_t LayerIndex, uint_fast64_t NodeMortonCode, int32& NodeIndex) const
{
	const auto& OctreeLayer = HieOctree.Layers[LayerIndex];
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

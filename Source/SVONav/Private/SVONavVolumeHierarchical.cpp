// Fill out your copyright notice in the Description page of Project Settings.


#include "SVONavVolumeHierarchical.h"

ASVONavVolumeHierarchical::ASVONavVolumeHierarchical(const FObjectInitializer& ObjectInitializer)
      : Super(ObjectInitializer)
{
}

void ASVONavVolumeHierarchical::InternalBuildOctree()
{
	InitRasterize();
	RasterizeLayer0();
	RasterizeLayer1();
	for (int32 i = 2; i < NumLayer; i++)
	{
	    RasterizeSparseLayer(i);
	}
	for (int32 i = 0; i < NumLayer; i ++)
	{
	    HierarchyStartIndex.Add(Octree.Layers[i].Num() - 1);
	}
	BuildLayer0Link(0);
	for (int32 i = 1; i < NumLayer; i++)
	{
	    BuildHierarchyNodes(i);
	    BuildLayerLink(i);
	}
}

void ASVONavVolumeHierarchical::InitRasterize()
{
	BlockedIndices.Emplace();

	for (int32 I = 0; I < GetLayerNodeCount(3); I++)
	{
		FVector Location;
		GetNodeLocation(3, I, Location);
		if (IsBlocked(Location, VoxelHalfSizes[3]))
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
	
	for (int32 I = 0; I < BlockedIndices.Num(); I++)
	{
		UE_LOG(LogTemp,Warning, TEXT("BlockedIndices: %i"), BlockedIndices[I].Num());
	}
	UE_LOG(LogTemp,Warning, TEXT("NodeCount: %i"), GetLayerNodeCount(3));
	UE_LOG(LogTemp,Warning, TEXT("Voxel HalfSize: %i"), VoxelHalfSizes[1]);
}

void ASVONavVolumeHierarchical::RasterizeLayer0()
{
	Octree.Layers.Emplace();
    int32 LeafIndex = 0;

	const int32 NumNodes = GetLayerNodeCount(2);

	for (int32 I = 0; I < NumNodes; I++)
	{
		if (BlockedIndices[0].Contains(I >> 3))
		{
			FVector NodeLocation;
			GetNodeLocation(2, I, NodeLocation);
			if (IsBlocked(NodeLocation, VoxelHalfSizes[2]))
			{
				const FVector Location = NodeLocation - VoxelHalfSizes[2];
				const float VoxelScale = VoxelHalfSizes[2] * 0.5f;
				for (int32 V = 0; V < 64; V++)
				{
					uint_fast32_t X, Y, Z;
					morton3D_64_decode(V, X, Y, Z);
					const FVector VoxelLocation = Location + FVector(X * VoxelScale, Y * VoxelScale, Z * VoxelScale) + VoxelScale *
                        0.5f;
					if (LeafIndex >= Octree.Leaves.Num() - 1) Octree.Leaves.AddDefaulted(1);

					if (IsBlocked(VoxelLocation, VoxelScale * 0.5f))
					{
						Octree.Leaves[LeafIndex].SetSubNode(V);
					}
					else
					{
						//create first layer of hierarchical octree
						int32 Index = Octree.Layers[0].Emplace();
						FSVONavNode& NewNode = Octree.Layers[0][Index];

						FIntVector Voxel;
						GetMortonVoxel(VoxelLocation, 0, Voxel);
						const uint_fast64_t MortonCode = morton3D_64_encode(Voxel.X, Voxel.Y, Voxel.Z);
						NewNode.MortonCode = MortonCode;
					}
				}
				LeafIndex++;
			}
			else
			{
				Octree.Leaves.AddDefaulted(1);
				LeafIndex++;
			}
		}
	}
}



void ASVONavVolumeHierarchical::RasterizeLayer1()
{
	Octree.Layers.Emplace();
	TArray<mortoncode_t> UniqueParentCodes;
	for (int32 I = 0; I < Octree.Layers[0].Num(); I++)
	{
		FSVONavNode& Child = Octree.Layers[0][I];
		mortoncode_t ParentCode = Child.MortonCode >> 3;
		if (!UniqueParentCodes.Contains(ParentCode))
		{
			UniqueParentCodes.Add(ParentCode);

			uint32 ParentIndex = Octree.Layers[1].Emplace();
			FSVONavNode& Parent = Octree.Layers[1][ParentIndex];
			Parent.MortonCode = ParentCode;
		}
	}
}

void ASVONavVolumeHierarchical::RasterizeSparseLayer(layerindex_t LayerIndex)
{
	Octree.Layers.Emplace();
	Octree.Layers[LayerIndex].Reserve(BlockedIndices[LayerIndex - 2].Num() * 8);
	const int32 NumNodes = GetLayerNodeCount(LayerIndex);
	for (int32 I = 0; I < NumNodes; I++)
	{
		if (BlockedIndices[LayerIndex - 2].Contains(I >> 3))
		{
			const int32 Index = Octree.Layers[LayerIndex].Emplace();
			FSVONavNode& NewNode = Octree.Layers[LayerIndex][Index];
			NewNode.MortonCode = I;
		}
	}
}

void ASVONavVolumeHierarchical::BuildLayer0Link(layerindex_t LayerIndex)
{
	if (Octree.Layers.Num() == 0) return;
	TArray<FSVONavNode>& LayerNodes = Octree.Layers[LayerIndex];

	for (int32 I = 0; I < LayerNodes.Num(); I++)
	{
		FSVONavNode& Node = LayerNodes[I];

		FVector NodeLocation;
		GetNodeLocation(LayerIndex, Node.MortonCode, NodeLocation);

		for (int32 Direction = 0; Direction < 6; Direction++)
		{
			//int32 NodeIndex = I;
			mortoncode_t CurrentCode = Node.MortonCode;
			mortoncode_t OriginalCode = CurrentCode;
			FSVONavLink& Link = Node.Neighbours[Direction];
			uint8 CurrentLayer = LayerIndex;
			//FindLinkViaCode_Hie(CurrentLayer, CurrentCode, Direction, Link, NodeLocation);
			while (!FindLinkViaCode(CurrentLayer, CurrentCode, OriginalCode, Direction, Link, NodeLocation) &&
                CurrentLayer < Octree.Layers.Num() - 2)
			{
				CurrentLayer++;
				CurrentCode = CurrentCode >> 3;
			}
			//Node.Neighbours[Direction + 6].Invalidate();
		}
	}
}



void ASVONavVolumeHierarchical::BuildHierarchyNodes(layerindex_t Layer)
 {
 	layerindex_t ChildLayer = Layer - 1;
 	TArray<mortoncode_t> UniqueParentCodes;
 
 	for (int32 I = 0; I < Octree.Layers[ChildLayer].Num(); I++)
 	{
 		FSVONavNode& Child = Octree.Layers[ChildLayer][I];
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
 			for (int32 A = 0; A < 8; A++) ChildIndexes.Append(GetArrayNodeIndex(ChildLayer, ChildCodes[A]));
 
 			for (int32 V = 0; V < ChildIndexes.Num(); V++)
 				Octree.Layers[ChildLayer][ChildIndexes[V]].Parent.SetLayerIndex(Layer);
 			
 			TArray<TArray<int32>> ChildGroups;
 			for (int32 B = 0; B < ChildIndexes.Num(); B++)
 			{
 				int32 ChildIndex = ChildGroups.Emplace();
 				ChildGroups[ChildIndex].Add(ChildIndexes[B]);
 				for (int32 C = 0; C < 12; C++)
 				{
 					FSVONavLink& NeighbourLink = Octree.Layers[ChildLayer][ChildIndexes[B]].Neighbours[C];
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
 						ParentIndex = Octree.Layers[Layer].Emplace();
 					}
 				}
 				else
 				{
 					ParentIndex = Octree.Layers[Layer].Emplace();
 				}
 				FSVONavNode& Parent = Octree.Layers[Layer][ParentIndex];
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
 					Octree.Layers[ChildLayer][Region[J][M]].Parent.SetNodeIndex(ParentIndex);
 					
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

void ASVONavVolumeHierarchical::BuildLayerLink(layerindex_t LayerIndex)
{
	if (Octree.Layers.Num() == 0) return;
	TArray<FSVONavNode>& LayerNodes = Octree.Layers[LayerIndex];

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
				const FSVONavNode& Child = GetNode(Node.Childs[C]);
				for (int32 A = 0; A < 12; A ++)
				{
					const FSVONavLink& ChildNeighbourLink = Child.Neighbours[A];
					if (ChildNeighbourLink.IsValid())
					{
						const FSVONavNode& ChildNeighbourNode = GetNode(ChildNeighbourLink);
						FSVONavLink InitLink;
						if (ChildNeighbourLink.GetLayerIndex() >= LayerIndex && ChildNeighbourLink.GetLayerIndex() < NumLayer -1)
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
						GetNodeLocation(LayerIndex, Node.MortonCode, NodeLocation);

						FVector NeighbourLocation;
						GetNodeLocation(Node.Neighbours[V].GetLayerIndex(),
                                            GetNode(Node.Neighbours[V]).MortonCode, NeighbourLocation);
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
			GetNodeLocation(LayerIndex, Node.MortonCode, NodeLocation);
			if (!IsBlocked(NodeLocation, VoxelHalfSizes[LayerIndex]))
			{
				for (int32 Direction = 0; Direction < 6; Direction++)
				{
					mortoncode_t CurrentCode = Node.MortonCode;
					mortoncode_t OriginalCode = CurrentCode;
					FSVONavLink& Link = Node.Neighbours[Direction];
					uint8 CurrentLayer = LayerIndex;

					while (!FindLinkViaCodeChildlessNode(CurrentLayer, CurrentCode, OriginalCode, Direction, Link,
					                                         NodeLocation) &&
						CurrentLayer < Octree.Layers.Num() - 2)
					{
						CurrentLayer++;
						CurrentCode = CurrentCode >> 3;
					}
				}
			}
		}
	}
}

TArray<int32> ASVONavVolumeHierarchical::GetArrayNodeIndex(layerindex_t LayerIndex, uint_fast64_t NodeMortonCode)
{
	TArray<int32> NodeSearchResults;
	const auto& OctreeLayer = Octree.Layers[LayerIndex];

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
			NodeSearchResults.Append(GetArrayNodeIndexExtra(LayerIndex, NodeMortonCode));
			return NodeSearchResults;
		}
		else End = Mean - 1;
		Mean = (Start + End) * 0.5f;
	}
	return NodeSearchResults;
}

TArray<int32> ASVONavVolumeHierarchical::GetArrayNodeIndexExtra(layerindex_t LayerIndex, uint_fast64_t NodeMortonCode)
{
	TArray<int32> NodeSearchResults;
	const auto& OctreeLayer = Octree.Layers[LayerIndex];

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

bool ASVONavVolumeHierarchical::FindLinkViaCodeChildlessNode(layerindex_t LayerIndex, mortoncode_t MortonCode,
                                                     mortoncode_t OriginalCode, uint8 Direction, FSVONavLink& Link,
                                                     const FVector& NodeLocation)
{
	const int32 MaxCoordinate = GetSegmentNodeCount(LayerIndex);
	TArray<FSVONavNode>& LayerNodes = Octree.Layers[LayerIndex];

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
		GetNodeLocation(LayerIndex, morton3D_64_encode(X, Y, Z), Location);
		if (IsBlocked(Location, VoxelHalfSizes[LayerIndex]))
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
			GetNodeLocation(LayerIndex, LayerNodes[NeighbourNodeIndex].MortonCode, NeighbourLocation);
			DebugLinks.Add(FSVONavDebugLink(NodeLocation, NeighbourLocation, LayerIndex));
		}
#endif

		return true;
	}

	Link.Invalidate();
	return false;
}

bool ASVONavVolumeHierarchical::FindLinkViaCode(layerindex_t LayerIndex, mortoncode_t MortonCode, mortoncode_t OriginalCode,
                                        uint8 Direction,
                                        FSVONavLink& Link, const FVector& NodeLocation)
{
	const int32 MaxCoordinate = GetSegmentNodeCount(LayerIndex);
	TArray<FSVONavNode>& LayerNodes = Octree.Layers[LayerIndex];

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
			GetNodeLocation(LayerIndex, morton3D_64_encode(X, Y, Z), Location);
			if (IsBlocked(Location, VoxelHalfSizes[LayerIndex]))
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
			GetNodeLocation(LayerIndex, LayerNodes[NeighbourNodeIndex].MortonCode, NeighbourLocation);
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

void ASVONavVolumeHierarchical::RegenerateLinkForDebug()
{
	for (int32 i = 1; i < NumLayer; i++)
	{
		BuildLayerLink(i);
	}
}

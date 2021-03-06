#pragma once

#include "CoreMinimal.h"
#include "SVONav/Private/libmorton/morton.h"
#include "NavigationSystem/Public/NavigationData.h"
#include "SVONavDefines.h"

#include "SVONavType.generated.h"
//OCTREE
struct SVONAV_API FSVOHieLink
{
	unsigned int LayerIndex:4;
	unsigned int NodeIndex:22;

	FSVOHieLink() :
        LayerIndex(15),
        NodeIndex(0)
	{
	}

	FSVOHieLink(const uint8 LayerIndex, const uint_fast32_t NodeIndex, const uint8 SubNodeIndex) :
        LayerIndex(LayerIndex),
        NodeIndex(NodeIndex)
	{
	}

	uint8 GetLayerIndex() const { return LayerIndex; }
	void SetLayerIndex(const uint8 NewLayerIndex) { LayerIndex = NewLayerIndex; }

	uint_fast32_t GetNodeIndex() const { return NodeIndex; }
	void SetNodeIndex(const uint_fast32_t NewNodeIndex) { NodeIndex = NewNodeIndex; }

	bool IsValid() const { return LayerIndex != 15; }
	void Invalidate() { LayerIndex = 15; }

	bool operator==(const FSVOHieLink& OtherLink) const
	{
		return memcmp(this, &OtherLink, sizeof(FSVOHieLink)) == 0;
	}

	bool operator!=(const FSVOHieLink& OtherLink) const { return !(*this == OtherLink); }
	static FSVOHieLink GetInvalidLink() { return FSVOHieLink(15, 0, 0); }
	FString ToString() const { return FString::Printf(TEXT("%i:%i"), LayerIndex, NodeIndex); }
};

FORCEINLINE uint32 GetTypeHash(const FSVOHieLink& Link) { return *(uint32*)&Link; }

FORCEINLINE FArchive& operator <<(FArchive& Archive, FSVOHieLink& Link)
{
	Archive.Serialize(&Link, sizeof(FSVOHieLink));
	return Archive;
}

struct SVONAV_API FSVONavLink
{
	unsigned int LayerIndex:4;
	unsigned int NodeIndex:22;
	unsigned int SubNodeIndex:6;

	FSVONavLink() :
		LayerIndex(15),
		NodeIndex(0),
		SubNodeIndex(0)
	{
	}

	FSVONavLink(const uint8 LayerIndex, const uint_fast32_t NodeIndex, const uint8 SubNodeIndex) :
		LayerIndex(LayerIndex),
		NodeIndex(NodeIndex),
		SubNodeIndex(SubNodeIndex)
	{
	}

	uint8 GetLayerIndex() const { return LayerIndex; }
	void SetLayerIndex(const uint8 NewLayerIndex) { LayerIndex = NewLayerIndex; }

	uint_fast32_t GetNodeIndex() const { return NodeIndex; }
	void SetNodeIndex(const uint_fast32_t NewNodeIndex) { NodeIndex = NewNodeIndex; }

	uint8 GetSubNodeIndex() const { return SubNodeIndex; }
	void SetSubNodeIndex(const uint8 NewSubIndex) { SubNodeIndex = NewSubIndex; }

	bool IsValid() const { return LayerIndex != 15; }
	void Invalidate() { LayerIndex = 15; }

	bool operator==(const FSVONavLink& OtherLink) const
	{
		return memcmp(this, &OtherLink, sizeof(FSVONavLink)) == 0;
	}

	bool operator!=(const FSVONavLink& OtherLink) const { return !(*this == OtherLink); }
	static FSVONavLink GetInvalidLink() { return FSVONavLink(15, 0, 0); }
	FString ToString() const { return FString::Printf(TEXT("%i:%i:%i"), LayerIndex, NodeIndex, SubNodeIndex); }
};

FORCEINLINE uint32 GetTypeHash(const FSVONavLink& Link) { return *(uint32*)&Link; }

FORCEINLINE FArchive& operator <<(FArchive& Archive, FSVONavLink& Link)
{
	Archive.Serialize(&Link, sizeof(FSVONavLink));
	return Archive;
}

struct SVONAV_API FSVONavLeafNode
{
	uint_fast64_t SubNodes = 0;

	bool GetSubNodeAt(uint_fast32_t X, uint_fast32_t Y, uint_fast32_t Z) const
	{
		const uint_fast64_t MortonCode = 0;
		morton3D_64_decode(MortonCode, X, Y, Z);
		return (SubNodes & 1ULL << morton3D_64_encode(X, Y, Z)) != 0;
	}

	void SetSubNodeAt(uint_fast32_t X, uint_fast32_t Y, uint_fast32_t Z)
	{
		const uint_fast64_t MortonCode = 0;
		morton3D_64_decode(MortonCode, X, Y, Z);
		SubNodes |= 1ULL << morton3D_64_encode(X, Y, Z);
	}

	void SetSubNode(const uint8 Index) { SubNodes |= 1ULL << Index; }
	bool GetSubNode(const uint_fast64_t MortonCode) const { return (SubNodes & 1ULL << MortonCode) != 0; }
	void ClearSubNode(const uint8 Index) { SubNodes &= !(1ULL << Index); }
	bool IsOccluded() const { return SubNodes == -1; }
	bool IsEmpty() const { return SubNodes == 0; }
};

FORCEINLINE FArchive& operator<<(FArchive& Archive, FSVONavLeafNode& Leaf)
{
	Archive << Leaf.SubNodes;
	return Archive;
}

struct SVONAV_API FSVOHieNode
{
	mortoncode_t MortonCode;
	FSVONavLink Parent;
	FSVONavLink FirstChild;
	TArray<FSVONavLink> Neighbours;
	TArray<FSVONavLink> Children;

	FSVOHieNode() :
        MortonCode(0),
        Parent(FSVONavLink::GetInvalidLink()),
        FirstChild(FSVONavLink::GetInvalidLink())
	{
	}

	bool HasChildren() const { return FirstChild.IsValid(); }

	int32 GetChildNum() const {return Children.Num();}

	int32 GetNeighbourNum() const {return Neighbours.Num();}

	bool operator==(const FSVOHieNode& Node) const{return MortonCode == Node.MortonCode;}
};

FORCEINLINE FArchive& operator <<(FArchive& Ar, FSVOHieNode& Node)
{
	Ar << Node.MortonCode;
	Ar << Node.Parent;
	Ar << Node.FirstChild;
	Ar << Node.Neighbours;
	Ar << Node.Children;
	return Ar;
}

struct SVONAV_API FSVONavNode
{
	mortoncode_t MortonCode;
	FSVONavLink Parent;
	FSVONavLink FirstChild;
	FSVONavLink Neighbours[6];
	TArray<FSVONavLink> NeighbourSet;
	TArray<FSVONavLink> Children;

	FSVONavNode() :
		MortonCode(0),
		Parent(FSVONavLink::GetInvalidLink()),
		FirstChild(FSVONavLink::GetInvalidLink())
	{
	}

	bool HasChildren() const { return FirstChild.IsValid(); }

	int32 GetChildNum() const {return Children.Num();}

	int32 GetNeighbourNum() const{return NeighbourSet.Num();}

	bool operator==(const FSVONavNode& Node) const
	{
		return MortonCode == Node.MortonCode;
	}
};

FORCEINLINE FArchive& operator <<(FArchive& Ar, FSVONavNode& Node)
{
	Ar << Node.MortonCode;
	Ar << Node.Parent;
	Ar << Node.FirstChild;

	for (int32 I = 0; I < 6; I++)
	{
		Ar << Node.Neighbours[I];
	}
	Ar << Node.NeighbourSet;
	Ar << Node.Children;

	return Ar;
}

struct SVONAV_API FSVONavOctree
{
	TArray<TArray<FSVONavNode>> Layers;
	TArray<FSVONavLeafNode> Leaves;

	void Reset()
	{
		Layers.Empty();
		Leaves.Empty();
	}

	int32 GetSize()
	{
		int Size = 0;
		Size += Leaves.Num() * sizeof(FSVONavLeafNode);
		for (int32 I = 0; I < Layers.Num(); I++)
		{
			Size += Layers[I].Num() * sizeof(FSVONavNode);
		}
		return Size;
	}
};

FORCEINLINE FArchive& operator<<(FArchive& Ar, FSVONavOctree& Octree)
{
	Ar << Octree.Layers;
	Ar << Octree.Leaves;
	return Ar;
}

struct SVONAV_API FSVOHieOctree
{
	TArray<TArray<FSVONavNode>> Layers;

	void Reset()
	{
		Layers.Empty();
	}

	int32 GetSize()
	{
		int Size = 0;
		for (int32 I = 0; I < Layers.Num(); I++)
		{
			Size += Layers[I].Num() * sizeof(FSVONavNode);
		}
		return Size;
	}
};

FORCEINLINE FArchive& operator<<(FArchive& Ar, FSVOHieOctree& Octree)
{
	Ar << Octree.Layers;
	return Ar;
}

//DEBUG

USTRUCT(BlueprintType)
struct SVONAV_API FSVONavDebugLocation
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite)
	FVector Location;

	UPROPERTY(BlueprintReadWrite)
	FColor Colour;

	UPROPERTY(BlueprintReadWrite)
	float LineScale;

	FSVONavDebugLocation(): Location(FVector::ZeroVector), Colour(FColor::Black), LineScale(0)
	{
	}
};

USTRUCT(BlueprintType)
struct SVONAV_API FSVONavDebugVoxel
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere)
	int32 Layer;

	UPROPERTY(EditAnywhere)
	int32 Index;

	FSVONavDebugVoxel(): Layer(0), Index(0)
	{
	}
};

struct SVONAV_API FSVONavDebugLink
{
	FVector Start;
	FVector End;
	uint8 LayerIndex;

	FSVONavDebugLink() :
		Start(FVector::ZeroVector),
		End(FVector::ZeroVector),
		LayerIndex(0)
	{
	}

	FSVONavDebugLink(const FVector Start, const FVector End, const uint8 LayerIndex) :
		Start(Start),
		End(End),
		LayerIndex(LayerIndex)
	{
	}
};

FORCEINLINE FArchive& operator<<(FArchive& Ar, FSVONavDebugLink& DebugLink)
{
	Ar << DebugLink.Start;
	Ar << DebugLink.End;
	Ar << DebugLink.LayerIndex;
	return Ar;
}

USTRUCT(BlueprintType)
struct SVONAV_API FSVONavDebugPath
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite)
	TArray<FVector> Points;

	UPROPERTY(BlueprintReadWrite)
	FColor Color;

	UPROPERTY(BlueprintReadWrite)
	float LineScale;

	FSVONavDebugPath() : Color(FColor::Black), LineScale(0)
	{
	}
};

//NAVIGATION

USTRUCT(BlueprintType)
struct SVONAV_API FSVONavPathPoint
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite)
	FVector Location;

	UPROPERTY(BlueprintReadWrite)
	int32 Layer;

	UPROPERTY(BlueprintReadWrite)
	int32 Index;

	UPROPERTY(BlueprintReadWrite)
	bool Refined;

	FSVONavPathPoint() :
		Location(FVector::ZeroVector),
		Layer(-1),
		Index(-1),
		Refined(false)
	{
	}

	FSVONavPathPoint(const FVector& Location, const int32 LayerIndex, const int32 NodeIndex, const bool bRefine) :
		Location(Location),
		Layer(LayerIndex),
		Index(NodeIndex),
		Refined(bRefine)
	{
	}

	FSVONavLink GetLink()
	{
		FSVONavLink Link;
		Link.SetLayerIndex(Layer);
		Link.SetNodeIndex(Index);
		return Link;
	}
};

USTRUCT(BlueprintType)
struct SVONAV_API FSVONavPath
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite)
	TArray<FSVONavPathPoint> Points;

	void Add(const FSVONavPathPoint& Point) { Points.Add(Point); }
	void Empty() { Points.Empty(); }
	TArray<FSVONavPathPoint>& GetPoints() { return Points; }
	void SetPoints(const TArray<FSVONavPathPoint> NewPoints) { Points = NewPoints; }
	void GetPath(TArray<FVector>& Path) { for (const auto& Point : Points) { Path.Add(Point.Location); } }
	FSVONavLink GetLink(int32 PointIndex)
	{
		FSVONavLink Link = FSVONavLink();
		Link.SetLayerIndex(Points[PointIndex].Layer);
		Link.SetNodeIndex(Points[PointIndex].Index);
		return Link;
	}

	// Copy the path positions into a standard navigation path
	void CreateNavPath(FNavigationPath& OutPath)
	{
		for (const FSVONavPathPoint& Point : Points)
		{
			OutPath.GetPathPoints().Add(Point.Location);
		}
	}
};

typedef TSharedPtr<FSVONavPath, ESPMode::ThreadSafe> FSVONavPathSharedPtr;

UENUM()
enum class ESVONavAlgorithm: uint8
{
	HierarchicalAStar UMETA(DisplayName="Hierarchical A*"),
	GreedyAStar UMETA(DisplayName="Greedy A*"),
	Testing UMETA(DisplayName="Testing")
};

UENUM()
enum class ESVONavHeuristic: uint8
{
	Manhattan UMETA(DisplayName="Manhattan"),
	Euclidean UMETA(DisplayName="Euclidean")
};

UENUM()
enum class ESVONavPathFindingCallResult: uint8
{
	Success UMETA(DisplayName="Call Success", ToolTip="Find path task was called successfully."),
	Reachable UMETA(DisplayName="Target in line of sight", ToolTip="Find path unnecessary. Target is already reachable."),
	NoVolume UMETA(DisplayName="Volume not found", ToolTip="SVONav component owner is not inside a SVONav volume."),
	NoOctree UMETA(DisplayName="Octree not found", ToolTip="SVONav octree has not been built."),
	NoStart UMETA(DisplayName="Start link not found", ToolTip="Failed to find start link."),
	NoTarget UMETA(DisplayName="Target link not found", ToolTip="Failed to find target link.")
};

UENUM()
enum class ESVONavFindLineOfSightCallResult: uint8
{
	Success UMETA(DisplayName="Call Success", ToolTip="Line of sight task was called successfully."),
	Visible UMETA(DisplayName="Target in line of sight", ToolTip="Find Line of sight unnecessary. Target is already visible."),
	NoVolume UMETA(DisplayName="Volume not found", ToolTip="SVONav component owner is not inside a SVONav volume."),
	NoOctree UMETA(DisplayName="Octree not found", ToolTip="SVONav octree has not been built."),
	NoStart UMETA(DisplayName="Start link not found", ToolTip="Failed to find start link."),
	NoTarget UMETA(DisplayName="Target link not found", ToolTip="Failed to find target link.")
};

UENUM()
enum class ESVONavPathPruning: uint8
{
	None UMETA(DisplayName="None", ToolTip="Do not use path pruning."),
	WithClearance UMETA(DisplayName="With clearance", ToolTip="Use path pruning with actor radius as clearance."),
	WithoutClearance UMETA(DisplayName="Without clearance", ToolTip="Use path pruning without clearance.")
};

USTRUCT(BlueprintType)
struct SVONAV_API FSVONavPathFindingConfig
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite)
	float EstimateWeight;

	UPROPERTY(BlueprintReadWrite)
	float NodeSizePreference;

	UPROPERTY(BlueprintReadWrite)
	ESVONavAlgorithm Algorithm;

	UPROPERTY(BlueprintReadWrite)
	ESVONavHeuristic Heuristic;

	UPROPERTY(BlueprintReadWrite)
	ESVONavPathPruning PathPruning;

	UPROPERTY(BlueprintReadWrite)
	int32 PathSmoothing;

	UPROPERTY(BlueprintReadWrite)
	bool UseUnitCost;

	UPROPERTY(BlueprintReadWrite)
	float UnitCost;

	FSVONavPathFindingConfig() :
		EstimateWeight(5.0f),
		NodeSizePreference(1.0f),
		Algorithm(ESVONavAlgorithm::HierarchicalAStar),
		Heuristic(ESVONavHeuristic::Euclidean),
		PathPruning(ESVONavPathPruning::None),
		PathSmoothing(3),
		UseUnitCost(false),
		UnitCost(1.0f)
	{
	}
};

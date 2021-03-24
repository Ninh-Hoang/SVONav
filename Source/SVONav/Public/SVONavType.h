#pragma once

#include "CoreMinimal.h"
#include "SVONav/Private/libmorton/morton.h"
#include "SVONavType.generated.h"

USTRUCT(BlueprintType)
struct SVONAV_API FSVONavPathPoint
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadWrite)
	FVector Location;

	UPROPERTY(BlueprintReadWrite)
	int32 Layer;

	FSVONavPathPoint() :
		Location(FVector::ZeroVector),
		Layer(-1)
	{
	}

	FSVONavPathPoint(const FVector& Location, const int32 LayerIndex) :
		Location(Location),
		Layer(LayerIndex)
	{
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
	TArray<FSVONavPathPoint> GetPoints() const { return Points; }
	void SetPoints(const TArray<FSVONavPathPoint> NewPoints) { Points = NewPoints; }
	void GetPath(TArray<FVector>& Path) { for (const FSVONavPathPoint Point : Points) { Path.Add(Point.Location); } }
};

typedef TSharedPtr<FSVONavPath, ESPMode::ThreadSafe> FSVONavPathSharedPtr;

struct SVONAV_API FSVONavLink
{
	uint8 LayerIndex:4;
	uint_fast32_t NodeIndex:22;
	uint8 SubNodeIndex:6;

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

	bool operator==(const FSVONavLink& OtherEdge) const
	{
		return memcmp(this, &OtherEdge, sizeof(FSVONavLink)) == 0;
	}

	bool operator!=(const FSVONavLink& OtherEdge) const { return !(*this == OtherEdge); }
	static FSVONavLink GetInvalidEdge() { return FSVONavLink(15, 0, 0); }
	FString ToString() const { return FString::Printf(TEXT("%i:%i:%i"), LayerIndex, NodeIndex, SubNodeIndex); }
};

FORCEINLINE uint32 GetTypeHash(const FSVONavLink& Edge) { return *(uint32*)&Edge; }

FORCEINLINE FArchive& operator <<(FArchive& Archive, FSVONavLink& Edge)
{
	Archive.Serialize(&Edge, sizeof(FSVONavLink));
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

struct SVONAV_API FSVONavNode
{
	uint_fast64_t MortonCode;
	FSVONavLink Parent;
	FSVONavLink FirstChild;
	FSVONavLink Neighbours[6];

	FSVONavNode() :
		MortonCode(0),
		Parent(FSVONavLink::GetInvalidEdge()),
		FirstChild(FSVONavLink::GetInvalidEdge())
	{
	}

	bool HasChildren() const { return FirstChild.IsValid(); }
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

FORCEINLINE FArchive& operator<<(FArchive& Ar, FSVONavDebugLink& DebugEdge)
{
	Ar << DebugEdge.Start;
	Ar << DebugEdge.End;
	Ar << DebugEdge.LayerIndex;
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

UENUM()
enum class ESVONavHeuristic: uint8
{
	Manhattan UMETA(DisplayName="Manhattan"),
	Euclidean UMETA(DisplayName="Euclidean")
};

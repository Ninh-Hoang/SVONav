// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SVONavType.h"
#include "GameFramework/Volume.h"
#include "SVONavVolume.generated.h"

DECLARE_DELEGATE(FSVONavUpdateOctreeDelegate);

/**
 * Volume contains the octree and methods required for  navigation
 */
UCLASS(Blueprintable, meta=(DisplayName = "SVO Navigation Volume"))
class SVONAV_API ASVONavVolume : public AVolume
{
	GENERATED_BODY()

public:
	ASVONavVolume(const FObjectInitializer& ObjectInitializer);

	// The size of the volume. This will be approximated in order to support the requested voxel size
	UPROPERTY(EditAnywhere, meta=(ClampMin = "0.000001"), DisplayName = "Desired Volume Size",
		Category = "SVONav|Volume")
	float VolumeSize = 200.0f;

	// The minimum size of a leaf voxel in the X, Y and Z dimensions.
	UPROPERTY(EditAnywhere, meta=(ClampMin = "0.000001"), DisplayName = "Minimum Voxel Size",
		Category = "SVONav|Volume")
	float VoxelSize = 200.0f;

	// Which collision channel to use for object tracing during octree generation
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Volume")
	TEnumAsByte<ECollisionChannel> CollisionChannel;

	// The minimum distance away from any object traces to apply during octree generation
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Volume")
	float Clearance = 0.f;

	// How often to tick this actor to perform dynamic updates
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Volume")
	float TickInterval = 0.2f;

	// Draw distance for debug lines
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Debugging")
	float DebugDistance = 20000.f;

	// Show the entire volume bounds
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Debugging")
	bool bDisplayVolumeBounds = false;

	// The colour for the volume bounds debug draw
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Debugging")
	FColor VolumeBoundsColour = FColor(0, 128, 0, 64);

	// Show the octree voxel node bounds
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Debugging|Voxels")
	bool bDisplayLayers = false;

	// Show the octree layer bounds
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Debugging|Voxels")
	bool bDisplayLeaves = false;

	// Show the occluded octree voxel sub-leaf bounds
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Debugging|Voxels")
	bool bDisplayLeafOcclusion = false;

	// Show adjacency link between each node
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Debugging|Voxels")
	bool bDisplayNeighbourLink = false;

	// The scaling factor for debug line drawing. Set to zero for fastest performance
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Debugging|Voxels")
	float LineScale = 0.0f;

	// The scaling factor for debug line drawing. Set to zero for fastest performance
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Debugging|Voxel")
	TArray<FSVONavDebugVoxel> DebugVoxelList;

	// The colours for debug drawing each layer. Colours added will be spread across a gradient
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Debugging|Voxels|Colours")
	TArray<FColor> LayerColours = {FColor::Magenta, FColor::Blue, FColor::Cyan, FColor::Green};

	// The colour for leaf occlusion debug drawing
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Debugging|Voxels|Colours")
	FColor LeafOcclusionColour = FColor::Yellow;

	// Show the morton codes within each voxel
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Debugging|Morton Codes")
	bool bDisplayMortonCodes = false;

	// The colour for morton code debug drawing
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Debugging|Morton Codes")
	FColor MortonCodeColour = FColor::Magenta;

	// The scaling factor for morton code debug drawing
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Debugging|Morton Codes")
	float MortonCodeScale = 1.0f;

	// The number of voxel subdivisions calculated to meet the desired voxel size. Read-only
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SVONav|Info")
	int32 VoxelExponent = 6;

	// The actual volume size calculated to meet the desired volume size and voxel size. Read-only
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SVONav|Info")
	float ActualVolumeSize = 200.0f;

	// The number of layers created by the octree generation. Read-only
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SVONav|Info")
	uint8 NumLayers = 0;

	// The number of bytes used to store the generated octree. Read-only
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "SVONav|Info")
	int32 NumBytes = 0;

	UFUNCTION()
	void UpdateTaskComplete();

#if WITH_EDITOR
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
	virtual void PostEditUndo() override;
#endif

	virtual void OnConstruction(const FTransform& Transform) override;
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;
	virtual void PostRegisterAllComponents() override;
	virtual void PostUnregisterAllComponents() override;
	virtual void EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown,
	                                    bool bCtrlDown) override;
	virtual void
	EditorApplyRotation(const FRotator& DeltaRotation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
	virtual void EditorApplyScale(const FVector& DeltaScale, const FVector* PivotLocation, bool bAltDown,
	                              bool bShiftDown, bool bCtrlDown) override;
	virtual void Serialize(FArchive& Ar) override;

	void Initialise();
	bool BuildOctree();
	void UpdateOctree();
	
	//update octree
	void LockOctree() { bOctreeLocked = true; }
	void UnlockOctree() { bOctreeLocked = false; }
	
	//getter setter checker
	bool OctreeValid() const { return NumLayers > 0; }
	void GetVolumeExtents(const FVector& Location, int32 LayerIndex, FIntVector& Extents) const;
	void GetMortonVoxel(const FVector& Location, int32 LayerIndex, FIntVector& MortonLocation) const;
	void GetMortonVoxel_Hie(const FVector& Location, int32 LayerIndex, FIntVector& MortonLocation) const;
	FBox GetBoundingBox() const;
	bool GetLink(const FVector& Location, FSVONavLink& Link);
	bool GetLink_Hie(const FVector& Location, FSVONavLink& Link);
	const FSVONavLeafNode& GetLeafNode(nodeindex_t aIndex) const;
	bool FindAccessibleLink(FVector& Location, FSVONavLink& Link);
	float GetVoxelScale(uint8 LayerIndex) const;
	float GetVoxelScale_Hie(uint8 LayerIndex) const;
	const TArray<FSVONavNode>& GetLayer(uint8 LayerIndex) const { return Octree.Layers[LayerIndex]; };
	const TArray<FSVONavNode>& GetLayer_Hie(uint8 LayerIndex) const { return HieOctree.Layers[LayerIndex]; };
	const FSVONavNode& GetNode(const FSVONavLink& Link) const;
	const FSVONavNode& GetNode_Hie(const FSVONavLink& Link) const;
	bool LinkNodeIsValid(const FSVONavLink& Link) const;
	bool GetLinkLocation(const FSVONavLink& Link, FVector& Location) const;
	bool GetNodeLocation(uint8 LayerIndex, uint_fast64_t MortonCode, FVector& Location) const;
	bool GetNodeLocation_Hie(uint8 LayerIndex, uint_fast64_t MortonCode, FVector& Location) const;
	bool GetNodeLocation(const FSVONavLink& Link, FVector& Location);
	void GetNeighbourLeaves(const FSVONavLink& Link, TArray<FSVONavLink>& NeighbourLinks) const;
	void GetNeighbourLinks(const FSVONavLink& Link, TArray<FSVONavLink>& NeighbourLinks) const;
	void GetNeighbourLinks_Hie(const FSVONavLink& Link, TArray<FSVONavLink>& NeighbourLinks) const;
	bool IsWithinBounds(const FVector Location) const { return GetBoundingBox().IsInside(Location); }
	int32 GetLayerCount_Hie() const {return HieOctree.Layers.Num();}

	//debug draw
	void FlushDebugDraw() const;
	void AddDebugNavPath(const FSVONavDebugPath DebugPath);
	void RequestOctreeDebugDraw();
	void DebugDrawOctree();
	void DebugDrawOctree_Hie();
	FColor GetLayerColour(const int32 LayerIndex) const;
	FColor GetLayerColour_Hie(const int32 LayerIndex) const;

private:
	FSVONavOctree Octree;
	FSVONavOctree CachedOctree;
	TArray<float> VoxelHalfSizes;
	TArray<TSet<uint_fast64_t>> BlockedIndices;
	
	FSVONavOctree HieOctree;
	TArray<float> VoxelHalfSizes_Hie;
	TArray<TSet<uint_fast64_t>> BlockedIndicesHie;
	int32 VoxelExponent_Hie;
	int32 NumLayer_Hie;

	FVector VolumeOrigin;
	FVector VolumeExtent;
	FCollisionQueryParams CollisionQueryParams;

	FSVONavUpdateOctreeDelegate OnUpdateComplete;
	bool bOctreeLocked = false;
	bool bUpdateRequested;

	TArray<int32> HierarchyStartIndex;
	
#if WITH_EDITOR
	bool bDebugDrawRequested;
	TArray<FSVONavDebugLink> DebugLinks;
	TArray<FSVONavDebugPath> DebugPaths;
	TArray<FSVONavDebugLocation> DebugLocations;
#endif

	const FIntVector Directions[6] = {
		FIntVector(1, 0, 0),
		FIntVector(-1, 0, 0),
		FIntVector(0, 1, 0),
		FIntVector(0, -1, 0),
		FIntVector(0, 0, 1),
		FIntVector(0, 0, -1)
	};

	const FIntVector FirstChildOffsetDir[7] = {
		FIntVector(1, 0, 0),
        FIntVector(1, 1, 0),
        FIntVector(0, 1, 0),
        FIntVector(0, 1, 1),
        FIntVector(1, 1, 1),
        FIntVector(1, 0, 1),
		FIntVector(0, 0, 1)
	};

	const int32 NodeOffsets[6][4] = {
		{0, 4, 2, 6},
		{1, 3, 5, 7},
		{0, 1, 4, 5},
		{2, 3, 6, 7},
		{0, 1, 2, 3},
		{4, 5, 6, 7}
	};
	
	const int32 LeafOffsets[6][16] = {
		{0, 2, 16, 18, 4, 6, 20, 22, 32, 34, 48, 50, 36, 38, 52, 54},
		{9, 11, 25, 27, 13, 15, 29, 31, 41, 43, 57, 59, 45, 47, 61, 63},
		{0, 1, 8, 9, 4, 5, 12, 13, 32, 33, 40, 41, 36, 37, 44, 45},
		{18, 19, 26, 27, 22, 23, 30, 31, 50, 51, 58, 59, 54, 55, 62, 63},
		{0, 1, 8, 9, 2, 3, 10, 11, 16, 17, 24, 25, 18, 19, 26, 27},
		{36, 37, 44, 45, 38, 39, 46, 47, 52, 53, 60, 61, 54, 55, 62, 63}
	};

	//octree generate
	void UpdateVolume();
	void InitRasterize();
	void RasterizeLayer(layerindex_t LayerIndex);
	void RasterizeLeaf(FVector NodeLocation, int32 LeafIndex);
	void BuildLinks(layerindex_t LayerIndex);
	void BuildLinks_Hie(layerindex_t LayerIndex);
	void BuildSecondsLinks_Hie(layerindex_t LayerIndex);
	void BuildHierarchyNodes_Hie(layerindex_t LayerIndex);

	//low res navmap generation
	void BuildHieOctree();
	void RasterizeFirstLayer_Hie();
	void RasterizSparseLayer_Hie(layerindex_t LayerIndex);
	void RasterizeLayer_Hie(layerindex_t Layer);
	bool FindLink_Hie(layerindex_t LayerIndex, int32 NodeIndex, uint8 Direction, FSVONavLink& Link, const FVector& NodeLocation);
	bool FindLinkViaCode_Hie(layerindex_t LayerIndex, mortoncode_t MortonCode, mortoncode_t OriginalCode, uint8 Direction, FSVONavLink& Link, const FVector& NodeLocation);
	bool FindLinkViaCodeChildlessNode_Hie(layerindex_t LayerIndex, mortoncode_t MortonCode, mortoncode_t OriginalCode, uint8 Direction, FSVONavLink& Link, const FVector& NodeLocation);
	bool CombineChildLink_Hie(layerindex_t LayerIndex, int32 NodeIndex, uint8 Direction, FSVONavLink& Link, const FVector& NodeLocation);
	bool GetNodeIndex_Hie(layerindex_t LayerIndex, uint_fast64_t NodeMortonCode, int32& NodeIndex) const;
	TArray<int32> GetArrayNodeIndex_Hie(layerindex_t LayerIndex, uint_fast64_t NodeMortonCode);
	TArray<int32> GetArrayNodeIndex_HieExtra(layerindex_t LayerIndex, uint_fast64_t NodeMortonCode);
	
	//getter setter checker
	TArray<FSVONavNode>& GetLayer(const layerindex_t LayerIndex) { return Octree.Layers[LayerIndex]; };
	float GetActualVolumeSize() const { return FMath::Pow(2, VoxelExponent) * (VoxelSize * 4); }
	int32 GetLayerNodeCount(layerindex_t LayerIndex) const;
	int32 GetLayerNodeCount_Hie(layerindex_t LayerIndex) const;
	int32 GetSegmentNodeCount(layerindex_t LayerIndex) const;
	int32 GetSegmentNodeCount_Hie(layerindex_t LayerIndex) const;
	bool InDebugRange(FVector Location) const;
	bool GetNodeIndex(layerindex_t LayerIndex, uint_fast64_t NodeMortonCode, int32& NodeIndex) const;
	int32 GetInsertIndex(layerindex_t LayerIndex, uint_fast64_t MortonCode) const;
	bool FindLink(layerindex_t LayerIndex, int32 NodeIndex, uint8 Direction, FSVONavLink& Link, const FVector& NodeLocation);
	bool IsBlocked(const FVector& Location, float Size) const;
	bool IsBlocked(const FVector& Location, float Size, TArray<FOverlapResult>& OverlapResults) const;
	bool GetIndexForCode(layerindex_t aLayer, mortoncode_t aCode, nodeindex_t& oIndex) const;
	bool IsAnyMemberBlocked(layerindex_t aLayer, mortoncode_t aCode) const;
	
	//debug draw
	void DebugDrawVolume() const;
	void DebugDrawVoxel(FVector Location, FVector Extent, FColor Colour) const;
	void DebugDrawSphere(const FVector Location, const float Radius, const FColor Colour) const;
	void DebugDrawMortonCode(FVector Location, FString String, FColor Colour) const;
	void DebugDrawLeafOcclusion();
	void DebugDrawNeighbourLink() const;
	void DebugDrawBoundsMesh(FBox Box, FColor Colour) const;
};

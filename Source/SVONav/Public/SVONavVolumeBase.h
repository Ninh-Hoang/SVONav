#pragma once

#include "CoreMinimal.h"
#include "SVONavType.h"
#include "GameFramework/Volume.h"
#include "SVONavVolume.generated.h"

DECLARE_DELEGATE(FSVONavUpdateOctreeDelegate);

/**
* Volume contains the octree and methods required for  navigation
*/

UCLASS(Blueprintable, meta=(DisplayName = "SVO Hierarchical Navigation Volume"))
class SVONAV_API ASVONavVolumeBase : public AVolume
{
	GENERATED_BODY()

public:
	ASVONavVolumeBase(const FObjectInitializer& ObjectInitializer);

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

	virtual void Initialise();
	virtual bool BuildOctree();
	virtual void UpdateOctree();

	//update octree
	void LockOctree() { bOctreeLocked = true; }
	void UnlockOctree() { bOctreeLocked = false; }

	//getter setter checker
	bool OctreeValid() const { return NumLayers > 0; }
	void GetVolumeExtents(const FVector& Location, int32 LayerIndex, FIntVector& Extents) const;
	void GetMortonVoxel(const FVector& Location, int32 LayerIndex, FIntVector& MortonLocation) const;
	FBox GetBoundingBox() const;
	bool GetLink(const FVector& Location, FSVONavLink& Link);
	float GetVoxelScale(uint8 LayerIndex) const;
	const TArray<FSVONavNode>& GetLayer_Hie(uint8 LayerIndex) const { return Octree.Layers[LayerIndex]; };
	const FSVONavNode& GetNode(const FSVONavLink& Link) const;
	bool LinkNodeIsValid(const FSVONavLink& Link) const;
	bool GetNodeLocation_Hie(uint8 LayerIndex, uint_fast64_t MortonCode, FVector& Location) const;
	void GetNeighbourLinks(const FSVONavLink& Link, TArray<FSVONavLink>& NeighbourLinks) const;
	int32 GetLayerCount_Hie() const {return Octree.Layers.Num();}
	bool IsWithinBounds(const FVector Location) const { return GetBoundingBox().IsInside(Location); }

	//debug draw
	void FlushDebugDraw() const;
	void AddDebugNavPath(const FSVONavDebugPath DebugPath);
	void RequestOctreeDebugDraw();
	virtual void DebugDrawOctree();
	FColor GetLayerColour(const int32 LayerIndex) const;

protected:
	FSVONavOctree Octree;
	FSVONavOctree CachedOctree;
	TArray<float> VoxelHalfSizes;
	TArray<TSet<uint_fast64_t>> BlockedIndices;
	int32 NumLayer;

	FVector VolumeOrigin;
	FVector VolumeExtent;

	FCollisionQueryParams CollisionQueryParams;

	FSVONavUpdateOctreeDelegate OnUpdateComplete;
	bool bOctreeLocked = false;
	bool bUpdateRequested;
	
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
	virtual void UpdateVolume();
	virtual void InternalBuildOctree();
	virtual bool FindLink(layerindex_t LayerIndex, int32 NodeIndex, uint8 Direction, FSVONavLink& Link, const FVector& NodeLocation);
	virtual bool GetNodeIndex_Hie(layerindex_t LayerIndex, uint_fast64_t NodeMortonCode, int32& NodeIndex) const;

	bool IsBlocked(const FVector& Location, float Size) const;
	bool IsBlocked(const FVector& Location, float Size, TArray<FOverlapResult>& OverlapResults) const;
	
	int32 GetLayerNodeCount_Hie(layerindex_t LayerIndex) const;
	int32 GetSegmentNodeCount_Hie(layerindex_t LayerIndex) const;
	virtual float GetActualVolumeSize() const { return FMath::Pow(2, VoxelExponent) * (VoxelSize); }

	//debug draw
	void DebugDrawVolume() const;
	void DebugDrawVoxel(FVector Location, FVector Extent, FColor Colour) const;
	void DebugDrawSphere(const FVector Location, const float Radius, const FColor Colour) const;
	void DebugDrawMortonCode(FVector Location, FString String, FColor Colour) const;
	void DebugDrawLeafOcclusion();
	void DebugDrawNeighbourLink() const;
	void DebugDrawBoundsMesh(FBox Box, FColor Colour) const;
};

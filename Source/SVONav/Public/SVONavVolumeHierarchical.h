// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SVONavVolumeBase.h"
#include "SVONavVolumeHierarchical.generated.h"

/**
 * 
 */
UCLASS(Blueprintable, meta=(DisplayName = "SVO Hierarchical Nav Volume"))
class SVONAV_API ASVONavVolumeHierarchical : public ASVONavVolumeBase
{
	GENERATED_BODY()

public:
	
	ASVONavVolumeHierarchical(const FObjectInitializer& ObjectInitializer);
	TArray<int32> GetArrayNodeIndex(layerindex_t LayerIndex, uint_fast64_t NodeMortonCode);
	TArray<int32> GetArrayNodeIndexExtra(layerindex_t LayerIndex, uint_fast64_t NodeMortonCode);
	
protected:
	virtual void Initialise() override;
	virtual void InternalBuildOctree() override;
	virtual void RegenerateLinkForDebug() override;
	virtual void Serialize(FArchive& Ar) override;
	virtual bool GetNodeIndex(layerindex_t LayerIndex, uint_fast64_t NodeMortonCode, int32& NodeIndex) const override;
	
private:
	TArray<int32> HierarchyStartIndex;
	TArray<TMap<mortoncode_t, TArray<int32>>> DuplicatedMortonMatrix;
	
	virtual void InitRasterize();
	void RasterizeLayer0();
	void RasterizeLayer1();
	void RasterizeSparseLayer(layerindex_t LayerIndex);
	void BuildLayer0Link(layerindex_t LayerIndex);
	void BuildLayerLink(layerindex_t LayerIndex);
	void BuildHierarchyNodes(layerindex_t LayerIndex);

	bool FindLinkViaCode(layerindex_t LayerIndex, mortoncode_t MortonCode, mortoncode_t OriginalCode, uint8 Direction, FSVONavLink& Link, const FVector& NodeLocation);
	bool FindLinkViaCodeChildlessNode(layerindex_t LayerIndex, mortoncode_t MortonCode, mortoncode_t OriginalCode, uint8 Direction, FSVONavLink& Link, const FVector& NodeLocation);
};

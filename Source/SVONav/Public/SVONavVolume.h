// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SVONavVolumeBase.h"
#include "SVONavVolume.generated.h"

/**
 * Volume contains the octree and methods required for  navigation
 */
UCLASS(Blueprintable, meta=(DisplayName = "SVO Navigation Volume"))
class SVONAV_API ASVONavVolume : public ASVONavVolumeBase
{
	GENERATED_BODY()

public:
	ASVONavVolume(const FObjectInitializer& ObjectInitializer);

protected:
	virtual void InternalBuildOctree() override;
	virtual void RegenerateLinkForDebug() override;
	virtual void UpdateVolume() override;

	virtual bool FindLink(layerindex_t LayerIndex, int32 NodeIndex, uint8 Direction, FSVONavLink& Link, const FVector& NodeLocation) override;
	virtual float GetActualVolumeSize() const override { return FMath::Pow(2, VoxelExponent) * (VoxelSize * 4); }
private:
	//octree generate
	void InitRasterize();
	void RasterizeLayer(layerindex_t LayerIndex);
	void RasterizeLeaf(FVector NodeLocation, int32 LeafIndex);
	void BuildLinks(layerindex_t LayerIndex);
};

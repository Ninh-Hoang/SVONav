// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SVONavType.h"
#include "Components/ActorComponent.h"
#include "SVONavComponent.generated.h"

DECLARE_DYNAMIC_DELEGATE_OneParam(FFindPathTaskCompleteDynamicDelegate, bool, bPathFound);
DECLARE_DYNAMIC_DELEGATE_OneParam(FFindLineOfSightTaskCompleteDynamicDelegate, bool, bLineOfSightFound);
DECLARE_DYNAMIC_DELEGATE_OneParam(FFindCoverTaskCompleteDynamicDelegate, bool, bLocationFound);

UCLASS(BlueprintType, Blueprintable, meta=(BlueprintSpawnableComponent, DisplayName="SVONavComponent"))
class SVONAV_API USVONavComponent : public UActorComponent
{
	friend class ASVONavVolume;

	GENERATED_BODY()

public:

	// The heuristic to use for scoring during pathfinding
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Pathfinding")
	ESVONavHeuristic Heuristic = ESVONavHeuristic::Manhattan;
	
	// Making this value greater than 1 will make the algorithm "greedy"
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Pathfinding")
	float HeuristicWeight = 5.0f;

	// Making this value greater than 1 will make the algorithm prefer larger-sized nodes
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Pathfinding")
	float NodeSizePreference = 1.0f;

	// The heuristic to use for scoring during pathfinding
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Pathfinding")
	ESVONavPathPruning PathPruning = ESVONavPathPruning::WithClearance;

	// Number of times to iterate Catmull-Rom smoothing on navigation paths 
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Pathfinding")
	int32 PathSmoothing = 5;

#if WITH_EDITOR
	// Whether to debug draw the pathfinding paths
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Debugging")
	bool bDebugDrawEnabled;

	// Whether to log the pathfinding task process in the console. 
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Debugging")
	bool bDebugLogPathfinding;
#endif
	
	FSVONavPathSharedPtr SVONavPath;
	
	// Sets default values for this component's properties
	USVONavComponent(const FObjectInitializer& ObjectInitializer);

	const ASVONavVolume* GetCurrentVolume() const { return Volume; }
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	void ExecutePathFinding(const FSVONavLink& StartEdge, const FSVONavLink& TargetEdge, const FVector& StartLocation, const FVector& TargetLocation, FSVONavPathFindingConfig Config, FSVONavPath& Path);
	float HeuristicScore(FSVONavLink StartEdge, FSVONavLink TargetEdge, FSVONavPathFindingConfig Config) const;
	void AddPathStartLocation(FSVONavPath& Path) const;
	void ApplyPathPruning(FSVONavPath& Path, const FSVONavPathFindingConfig Config) const;
	void ApplyPathLineOfSight(FSVONavPath& Path, AActor* Target, float MinimumDistance) const;
	static void ApplyPathSmoothing(FSVONavPath& Path, FSVONavPathFindingConfig Config);
	void RequestNavPathDebugDraw(const FSVONavPath Path) const;

	//for testing
	UFUNCTION(BlueprintCallable, Category = SVONav)
	void FindPathAsync(const FVector &StartLocation, const FVector &TargetLocation, TArray<FVector>& OutPathPoints);

	bool FindPathAsync(const FVector& aStartPosition, const FVector& aTargetPosition, FSVONNavPathSharedPtr* oNavPath);
protected:
	UPROPERTY()
	ASVONavVolume* Volume;
	
	// Called when the game starts
	virtual void BeginPlay() override;

public:
	bool VolumeContainsOctree() const;
	bool VolumeContainsOwner() const;
	bool FindVolume();
};

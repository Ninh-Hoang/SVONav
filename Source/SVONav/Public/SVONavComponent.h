// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "SVONavType.h"
#include "Components/ActorComponent.h"
#include "SVONavComponent.generated.h"

DECLARE_DYNAMIC_DELEGATE_OneParam(FFindPathTaskCompleteDynamicDelegate, bool, bPathFound);

DECLARE_DYNAMIC_DELEGATE_OneParam(FFindLineOfSightTaskCompleteDynamicDelegate, bool, bLineOfSightFound);

DECLARE_DYNAMIC_DELEGATE_OneParam(FFindCoverTaskCompleteDynamicDelegate, bool, bLocationFound);

UCLASS(ClassGroup = (Custom), meta=(BlueprintSpawnableComponent, DisplayName="SVONavComponent"))
class SVONAV_API USVONavComponent : public UActorComponent
{
	friend class ASVONavVolume;

	GENERATED_BODY()

public:
	// The Algorithm use for pathfinding
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Pathfinding")
	ESVONavAlgorithm Algorithm = ESVONavAlgorithm::GreedyAStar;
	
	// The heuristic to use for scoring during pathfinding
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Pathfinding")
	ESVONavHeuristic Heuristic = ESVONavHeuristic::Euclidean;

	// Making this value greater than 1 will make the algorithm "greedy"
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Pathfinding")
	float HeuristicWeight = 5.0f;

	// Making this value greater than 1 will make the algorithm prefer larger-sized nodes
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Pathfinding")
	float NodeSizePreference = 2.0f;

	// The heuristic to use for scoring during pathfinding
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Pathfinding")
	ESVONavPathPruning PathPruning = ESVONavPathPruning::WithClearance;

	// Number of times to iterate Catmull-Rom smoothing on navigation paths 
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Pathfinding")
	int32 PathSmoothing = 5;

	// Do we use unit cost 
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Pathfinding")
	bool bUseUnitCost = false;

	// Custom unit cost
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "SVONav|Pathfinding")
	float UnitCost = 5.0f;

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

	const ASVONavVolume* GetCurrentVolume() const { return Volume; } /**/
	virtual void TickComponent(float DeltaTime, ELevelTick TickType,
	                           FActorComponentTickFunction* ThisTickFunction) override;

	void FindPathAsync(const FVector& StartLocation, const FVector& TargetLocation, const bool bCheckLineOfSight,
	                   FThreadSafeBool& CompleteFlag,
	                   FSVONavPathSharedPtr* NavPath,
	                   /* const FFindPathTaskCompleteDynamicDelegate OnComplete,*/
	                   ESVONavPathFindingCallResult& Result);

	bool FindPathImmediate(const FVector& StartLocation, const FVector& TargetLocation,
	                       const bool bCheckLineOfSight, FSVONavPathSharedPtr* NavPath,
	                       ESVONavPathFindingCallResult& Result);

	UFUNCTION(BlueprintCallable, Category = "SVONav|Pathfinding")
	bool DoesPathExist(const FVector& StartLocation, const FVector& TargetLocation);

	FSVONavPathSharedPtr& GetPath() { return SVONavPath; }
	virtual FVector GetPawnPosition() const;

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

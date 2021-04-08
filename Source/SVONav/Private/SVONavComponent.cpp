// Fill out your copyright notice in the Description page of Project Settings.


#include "SVONavComponent.h"
#include "SVONavVolume.h"
#include "SVONavFindPathTask.h"
#include "Kismet/GameplayStatics.h"
#include "SVONavPathFinder.h"
#include "SVONav/SVONav.h"
#include "Kismet/KismetMathLibrary.h"

#include <Runtime/Engine/Public/DrawDebugHelpers.h>
#include <Runtime/Engine/Classes/GameFramework/Actor.h>
#include <Runtime/Engine/Classes/Kismet/GameplayStatics.h>
#include <Runtime/Engine/Classes/Components/LineBatchComponent.h>
#include <Runtime/NavigationSystem/Public/NavigationData.h>

USVONavComponent::USVONavComponent(const FObjectInitializer& ObjectInitializer)
{
	PrimaryComponentTick.bCanEverTick = true;
	SVONavPath = MakeShareable<FSVONavPath>(new FSVONavPath());
	bWantsInitializeComponent = true;
}

// Called when the game starts
void USVONavComponent::BeginPlay()
{
	Super::BeginPlay();

	// ...
}

// Called every frame
void USVONavComponent::TickComponent(float DeltaTime, ELevelTick TickType,
                                     FActorComponentTickFunction* ThisTickFunction)
{
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

	// ...
}

void USVONavComponent::FindPathAsync(const FVector& StartLocation, const FVector& TargetLocation,
                                     const bool bCheckLineOfSight, FThreadSafeBool& CompleteFlag,
                                     FSVONavPathSharedPtr* NavPath,
                                     /*const FFindPathTaskCompleteDynamicDelegate OnComplete,*/
                                     ESVONavPathFindingCallResult& Result)
{
FSVONavLink StartLink;
	FSVONavLink TargetLink;
	FVector LegalStart = StartLocation;
	FVector LegalTarget = TargetLocation;

	// Error checking before task start
	if (!VolumeContainsOctree() || !VolumeContainsOwner()) FindVolume();
	if (!VolumeContainsOwner())
	{
		Result = ESVONavPathFindingCallResult::NoVolume;

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Error,
		                                 TEXT(
			                                 "%s: Find path cannot initialise. Nav3D component owner is not inside a Nav3D volume"
		                                 ), *GetOwner()->GetName());
#endif

		return;
	}
	// Check that an octree has been found
	if (!VolumeContainsOctree())
	{
		Result = ESVONavPathFindingCallResult::NoOctree;

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Error,
		                                 TEXT("%s: Find path cannot initialise. Nav3D octree has not been built"),
		                                 *GetOwner()->GetName());
#endif

		return;
	}

	if (bCheckLineOfSight)
	{
		// If there is a line of sight plus clearance with the target then no path finding is required
		FCollisionQueryParams CollisionQueryParams;
		CollisionQueryParams.bTraceComplex = true;
		CollisionQueryParams.TraceTag = "Nav3DLineOfSightCheck";
		FHitResult HitResult;
		float Radius = FMath::Max(50.f, GetOwner()->GetComponentsBoundingBox(true).GetExtent().GetMax());
		GetWorld()->SweepSingleByChannel(
			HitResult,
			StartLocation,
			TargetLocation,
			FQuat::Identity,
			Volume->CollisionChannel,
			FCollisionShape::MakeSphere(Radius),
			CollisionQueryParams
		);

		if (!HitResult.bBlockingHit)
		{
			Result = ESVONavPathFindingCallResult::Reachable;

#if WITH_EDITOR
			if (bDebugLogPathfinding) UE_LOG(LogTemp, Error,
			                                 TEXT(
				                                 "%s: Find path unnecessary. Nav3D component owner has a clear line of sight to target"
			                                 ), *GetOwner()->GetName());
#endif

			return;
		}
	}

	if (!Volume->GetLink(StartLocation, StartLink))
	{
		Result = ESVONavPathFindingCallResult::NoStart;

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Warning, TEXT("%s: Failed to find start Link. Searching nearby..."),
		                                 *GetOwner()->GetName());
#endif

		if (!Volume->FindAccessibleLink(LegalStart, StartLink))
		{
#if WITH_EDITOR
			if (bDebugLogPathfinding) UE_LOG(LogTemp, Error, TEXT("%s: No accessible adjacent Link found"),
			                                 *GetOwner()->GetName());
#endif

			return;
		}

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Display, TEXT("%s: Found legal start location"),
		                                 *GetOwner()->GetName());
#endif
	}

	if (!Volume->GetLink(TargetLocation, TargetLink))
	{
		Result = ESVONavPathFindingCallResult::NoTarget;

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Warning, TEXT("%s: Failed to find target Link. Searching nearby..."),
		                                 *GetOwner()->GetName());
#endif

		if (!Volume->FindAccessibleLink(LegalTarget, TargetLink))
		{
#if WITH_EDITOR
			if (bDebugLogPathfinding) UE_LOG(LogTemp, Error, TEXT("%s: No accessible Links found near target"),
			                                 *GetOwner()->GetName());
#endif

			return;
		}

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Display, TEXT("%s: Found accessible target location"),
		                                 *GetOwner()->GetName());
#endif
	}

	FSVONavPathFindingConfig Config;
	Config.Algorithm = Algorithm;
	Config.Heuristic = Heuristic;
	Config.EstimateWeight = HeuristicWeight;
	Config.NodeSizePreference = NodeSizePreference;
	Config.PathPruning = PathPruning;
	Config.PathSmoothing = PathSmoothing;
	Config.UseUnitCost = bUseUnitCost;
	Config.UnitCost = UnitCost;


	(new FAutoDeleteAsyncTask<FSVONavFindPathTask>(
		*Volume,
		GetWorld(),
		this,
		StartLink,
		TargetLink,
		LegalStart,
		LegalTarget,
		Config,
		NavPath,
		CompleteFlag,
		/*OnComplete,*/
		bDebugDrawEnabled))->StartBackgroundTask();
	Result = ESVONavPathFindingCallResult::Success;

#if WITH_EDITOR
	if (bDebugLogPathfinding) UE_LOG(LogTemp, Display, TEXT("%s: Find path task called successfully"),
	                                 *GetOwner()->GetName());
#endif
}

bool USVONavComponent::FindPathImmediate(const FVector& StartLocation, const FVector& TargetLocation,
                                         const bool bCheckLineOfSight, FSVONavPathSharedPtr* NavPath,
                                         ESVONavPathFindingCallResult& Result)
{
	FSVONavLink StartLink;
	FSVONavLink TargetLink;
	FVector LegalStart = StartLocation;
	FVector LegalTarget = TargetLocation;

	// Error checking before task start
	if (!VolumeContainsOctree() || !VolumeContainsOwner()) FindVolume();
	if (!VolumeContainsOwner())
	{
		Result = ESVONavPathFindingCallResult::NoVolume;

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Error,
		                                 TEXT(
			                                 "%s: Find path cannot initialise. Nav3D component owner is not inside a Nav3D volume"
		                                 ), *GetOwner()->GetName());
#endif

		return false;
	}
	// Check that an octree has been found
	if (!VolumeContainsOctree())
	{
		Result = ESVONavPathFindingCallResult::NoOctree;

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Error,
		                                 TEXT("%s: Find path cannot initialise. Nav3D octree has not been built"),
		                                 *GetOwner()->GetName());
#endif

		return false;
	}

	if (bCheckLineOfSight)
	{
		// If there is a line of sight plus clearance with the target then no path finding is required
		FCollisionQueryParams CollisionQueryParams;
		CollisionQueryParams.bTraceComplex = true;
		CollisionQueryParams.TraceTag = "Nav3DLineOfSightCheck";
		FHitResult HitResult;
		float Radius = FMath::Max(50.f, GetOwner()->GetComponentsBoundingBox(true).GetExtent().GetMax());
		GetWorld()->SweepSingleByChannel(
			HitResult,
			StartLocation,
			TargetLocation,
			FQuat::Identity,
			Volume->CollisionChannel,
			FCollisionShape::MakeSphere(Radius),
			CollisionQueryParams
		);

		if (!HitResult.bBlockingHit)
		{
			Result = ESVONavPathFindingCallResult::Reachable;

#if WITH_EDITOR
			if (bDebugLogPathfinding) UE_LOG(LogTemp, Error,
			                                 TEXT(
				                                 "%s: Find path unnecessary. Nav3D component owner has a clear line of sight to target"
			                                 ), *GetOwner()->GetName());
#endif

			return false;
		}
	}

	if (!Volume->GetLink(StartLocation, StartLink))
	{
		Result = ESVONavPathFindingCallResult::NoStart;

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Warning, TEXT("%s: Failed to find start Link. Searching nearby..."),
		                                 *GetOwner()->GetName());
#endif

		if (!Volume->FindAccessibleLink(LegalStart, StartLink))
		{
#if WITH_EDITOR
			if (bDebugLogPathfinding) UE_LOG(LogTemp, Error, TEXT("%s: No accessible adjacent Link found"),
			                                 *GetOwner()->GetName());
#endif

			return false;
		}

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Display, TEXT("%s: Found legal start location"),
		                                 *GetOwner()->GetName());
#endif
	}

	if (!Volume->GetLink(TargetLocation, TargetLink))
	{
		Result = ESVONavPathFindingCallResult::NoTarget;

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Warning, TEXT("%s: Failed to find target Link. Searching nearby..."),
		                                 *GetOwner()->GetName());
#endif

		if (!Volume->FindAccessibleLink(LegalTarget, TargetLink))
		{
#if WITH_EDITOR
			if (bDebugLogPathfinding) UE_LOG(LogTemp, Error, TEXT("%s: No accessible Links found near target"),
			                                 *GetOwner()->GetName());
#endif

			return false;
		}

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Display, TEXT("%s: Found accessible target location"),
		                                 *GetOwner()->GetName());
#endif
	}

	FSVONavPath* Path = NavPath->Get();

	Path->Empty();
	
	FSVONavPathFindingConfig Config;
	Config.Algorithm = Algorithm;
	Config.Heuristic = Heuristic;
	Config.EstimateWeight = HeuristicWeight;
	Config.NodeSizePreference = NodeSizePreference;
	Config.PathPruning = PathPruning;
	Config.PathSmoothing = PathSmoothing;
	Config.UseUnitCost = bUseUnitCost;
	Config.UnitCost = UnitCost;

	SVONavPathFinder PathFinder(GetWorld(), this, *Volume, Config);

	int PathResult = PathFinder.FindPath(StartLink, TargetLink, LegalStart, LegalTarget, Config, NavPath);
	
	Result = ESVONavPathFindingCallResult::Success;

#if WITH_EDITOR
	if (bDebugLogPathfinding) UE_LOG(LogTemp, Display, TEXT("%s: Find path task called successfully"),
	                                 *GetOwner()->GetName());
	if(bDebugDrawEnabled) PathFinder.DrawDebug(GetWorld(), *Volume, NavPath);
#endif

	return true;
}

bool USVONavComponent::DoesPathExist(const FVector& StartLocation, const FVector& TargetLocation)
{
	FSVONavLink StartLink;
	FSVONavLink TargetLink;
	FVector LegalStart = StartLocation;
	FVector LegalTarget = TargetLocation;
	
	// Error checking before task start
	/*if (!VolumeContainsOctree() || !VolumeContainsOwner()) FindVolume();
	if (!VolumeContainsOwner())
	{

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Error,
                                         TEXT(
                                             "%s: Find path cannot initialise. Nav3D component owner is not inside a Nav3D volume"
                                         ), *GetOwner()->GetName());
#endif

		return false;
	}
	// Check that an octree has been found
	if (!VolumeContainsOctree())
	{

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Error,
                                         TEXT("%s: Find path cannot initialise. Nav3D octree has not been built"),
                                         *GetOwner()->GetName());
#endif

		return false;
	}*/

	if(!HieVolume)
	{
		if(!FindHierarchicalVolume()) return false;
	}

	if (!HieVolume->GetLink(StartLocation, StartLink))
	{

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Warning, TEXT("%s: Failed to find start Link. Searching nearby..."),
                                         *GetOwner()->GetName());
#endif
		return false;
		/*if (!Volume->FindAccessibleLink(LegalStart, StartLink))
		{
#if WITH_EDITOR
			if (bDebugLogPathfinding) UE_LOG(LogTemp, Error, TEXT("%s: No accessible adjacent Link found"),
                                             *GetOwner()->GetName());
#endif

			return false;
		}*/

#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Display, TEXT("%s: Found legal start location"),
                                         *GetOwner()->GetName());
#endif
	}

	if (!HieVolume->GetLink(TargetLocation, TargetLink))
	{
#if WITH_EDITOR
		if (bDebugLogPathfinding) UE_LOG(LogTemp, Warning, TEXT("%s: Failed to find target Link. Searching nearby..."),
                                         *GetOwner()->GetName());
#endif
		return false;
		/*if (!Volume->FindAccessibleLink(LegalTarget, TargetLink))
		{
#if WITH_EDITOR
			if (bDebugLogPathfinding) UE_LOG(LogTemp, Error, TEXT("%s: No accessible Links found near target"),
                                             *GetOwner()->GetName());
#endif
	
			return false;
		}*/
	}

	FSVONavNode StartNode;
	StartNode = HieVolume->GetNode(StartLink);
	
	FSVONavNode TargetNode;
	TargetNode = HieVolume->GetNode(TargetLink);
	
	int32 StartParentLayer = StartNode.Parent.GetLayerIndex();
	int32 TargetParentLayer = TargetNode.Parent.GetLayerIndex();

	int32 StartParentIndex = StartNode.Parent.GetNodeIndex();
	int32 TargetParentIndex = TargetNode.Parent.GetNodeIndex();

	int32 LayerNum = HieVolume->GetLayerCount();

	while(StartParentLayer != TargetParentLayer)
	{
		if(StartParentLayer < TargetParentLayer)
		{			
			StartNode = HieVolume->GetNode(StartNode.Parent);
			StartParentIndex = StartNode.Parent.GetNodeIndex();
			StartParentLayer = StartNode.Parent.GetLayerIndex();

		}
		else if (StartParentLayer > TargetParentLayer)
		{
			TargetNode = HieVolume->GetNode(TargetNode.Parent);
			TargetParentIndex = TargetNode.Parent.GetNodeIndex();
			TargetParentLayer = TargetNode.Parent.GetLayerIndex();
		}
		if(StartParentIndex == TargetParentIndex && StartParentLayer == TargetParentLayer)
		{
			UE_LOG(LogTemp, Warning, TEXT("Start Layer %i, Start Index %i"), StartParentLayer, StartParentIndex);
			UE_LOG(LogTemp, Warning, TEXT("Target Layer %i, Target Index %i"), TargetParentLayer, TargetParentIndex );
			return true;
		}
		
	}

	if(StartParentIndex  == TargetParentIndex)
	{
		UE_LOG(LogTemp, Warning, TEXT("Start Layer %i, Start Index %i"), StartParentLayer, StartParentIndex);
		UE_LOG(LogTemp, Warning, TEXT("Target Layer %i, Target Index %i"), TargetParentLayer, TargetParentIndex );
		return true;
	}

	
	while (StartParentIndex  != TargetParentIndex && StartParentLayer != 15 && TargetParentLayer != 15)
	{
		StartNode = HieVolume->GetNode(StartNode.Parent);
		StartParentIndex = StartNode.Parent.GetNodeIndex();
		StartParentLayer = StartNode.Parent.GetLayerIndex();
		TargetNode = HieVolume->GetNode(TargetNode.Parent);
		TargetParentIndex = TargetNode.Parent.GetNodeIndex();
		TargetParentLayer = TargetNode.Parent.GetLayerIndex();
		if(StartParentIndex  == TargetParentIndex && StartParentLayer != 15 && TargetParentLayer != 15)
		{
			UE_LOG(LogTemp, Warning, TEXT("Start Layer %i, Start Index %i"), StartParentLayer, StartParentIndex);
			UE_LOG(LogTemp, Warning, TEXT("Target Layer %i, Target Index %i"), TargetParentLayer, TargetParentIndex );
			return true;
		}
	}
	/*for(int i = 0; i < Volume->NumLayers; i++)
	{
		if(TargetNode == StartNode)
		{
			if (bDebugLogPathfinding) UE_LOG(LogTemp, Error, TEXT("%s: Path exist, confirmed after %i Iterations"), *GetOwner()->GetName(), i);
				return true;
		}

		if(!StartNode.Parent.IsValid()) break;
		StartNode = Volume->GetNode(StartNode.Parent);

		if(!TargetNode.Parent.IsValid()) break;
		TargetNode = Volume->GetNode(TargetNode.Parent);
	}*/
	return false;
}

FVector USVONavComponent::GetPawnPosition() const
{
	FVector Result;

	AController* Controller = Cast<AController>(GetOwner());

	if (Controller)
	{
		if (APawn* pawn = Controller->GetPawn())
			Result = pawn->GetActorLocation();
	}

	return Result;
}

bool USVONavComponent::VolumeContainsOctree() const
{
	return Volume && Volume->OctreeValid();
}

bool USVONavComponent::VolumeContainsOwner() const
{
	return GetOwner() && Volume->IsWithinBounds(GetOwner()->GetActorLocation());
}

bool USVONavComponent::FindVolume()
{
	TArray<AActor*> Volumes;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ASVONavVolume::StaticClass(), Volumes);

	for (auto& NavVolume : Volumes)
	{
		ASVONavVolume* CurrentVolume = Cast<ASVONavVolume>(NavVolume);
		if (CurrentVolume)
		{
			if (CurrentVolume->IsWithinBounds(GetOwner()->GetActorLocation()))
			{
				Volume = CurrentVolume;
				return true;
			}
		}
	}
	return false;
}

bool USVONavComponent::FindHierarchicalVolume()
{
	TArray<AActor*> Volumes;
	UGameplayStatics::GetAllActorsOfClass(GetWorld(), ASVONavVolumeBase::StaticClass(), Volumes);

	for (auto& NavVolume : Volumes)
	{
		ASVONavVolumeBase* CurrentVolume = Cast<ASVONavVolumeBase>(NavVolume);
		if (CurrentVolume)
		{
			if (CurrentVolume->IsWithinBounds(GetOwner()->GetActorLocation()))
			{
				HieVolume = CurrentVolume;
				return true;
			}
		}
	}
	return false;
}
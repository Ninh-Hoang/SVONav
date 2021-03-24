// Fill out your copyright notice in the Description page of Project Settings.


#include "SVONavComponent.h"
#include "SVONavVolume.h"
#include "Kismet/GameplayStatics.h"
#include "Kismet/KismetMathLibrary.h"

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

void USVONavComponent::ExecutePathFinding(const FSVONavLink& StartEdge, const FSVONavLink& TargetEdge,
                                          const FVector& StartLocation, const FVector& TargetLocation,
                                          FSVONavPathFindingConfig Config, FSVONavPath& Path)
{
}

float USVONavComponent::HeuristicScore(FSVONavLink StartEdge, FSVONavLink TargetEdge,
                                       FSVONavPathFindingConfig Config) const
{
}

void USVONavComponent::AddPathStartLocation(FSVONavPath& Path) const
{
}

void USVONavComponent::ApplyPathPruning(FSVONavPath& Path, const FSVONavPathFindingConfig Config) const
{
}

void USVONavComponent::ApplyPathLineOfSight(FSVONavPath& Path, AActor* Target, float MinimumDistance) const
{
}

void USVONavComponent::ApplyPathSmoothing(FSVONavPath& Path, FSVONavPathFindingConfig Config)
{
}

void USVONavComponent::RequestNavPathDebugDraw(const FSVONavPath Path) const
{
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

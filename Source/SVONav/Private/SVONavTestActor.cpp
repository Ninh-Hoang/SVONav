// Fill out your copyright notice in the Description page of Project Settings.


#include "SVONavTestActor.h"

#include "SVONavVolume.h"
#include "Components/LineBatchComponent.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/HUD.h"

// Sets default values
ASVONavTestActor::ASVONavTestActor()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = false;

	SphereComp = CreateDefaultSubobject<USphereComponent>(TEXT("DebugSphere"));
	NavComponent = CreateDefaultSubobject<USVONavComponent>(TEXT("NavComponent"));
	NavComponent->bDebugDrawEnabled = true;

	SphereComp->SetWorldScale3D(FVector(5));
	SphereComp->SetHiddenInGame(false);

	ThreadSafe = true;
}

// Called when the game starts or when spawned
void ASVONavTestActor::BeginPlay()
{
	Super::BeginPlay();
}

// Called every frame
void ASVONavTestActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
}

bool ASVONavTestActor::InitStartAndGoalActor()
{
	if (SVONavTestActor && SVONavTestActor != this)
	{
		SVONavTestActor->IsPathStart = IsPathStart ? false : true;
		SVONavTestActor->SVONavTestActor = this;
		return true;
	}
	return false;
}

void ASVONavTestActor::FindPath()
{
	if (!InitStartAndGoalActor()) return;
	if (IsPathStart)
	{
		/*if (ThreadSafe == true)
		{
			ThreadSafe = false;*/

			if (!FlushDebugLine()) return;

			ESVONavPathFindingCallResult Result;
			NavComponent->FindPathImmediate(this->GetActorLocation(),
			                            SVONavTestActor->GetActorLocation(),
			                            false,
			                            &NavComponent->GetPath(), Result);
		//}
	}
	else
	{
		SVONavTestActor->FindPath();
	}
}

bool ASVONavTestActor::FlushDebugLine()
{
	if (!GetWorld()) return false;
	if (GetWorld() && GetWorld()->PersistentLineBatcher)
	{
		GetWorld()->PersistentLineBatcher->Flush();
	}
	return true;
}

#if WITH_EDITOR
void ASVONavTestActor::EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown,
                                              bool bCtrlDown)
{
	Super::EditorApplyTranslation(DeltaTranslation, bAltDown, bShiftDown, bCtrlDown);
	FindPath();
}

void ASVONavTestActor::PostEditChangeProperty(FPropertyChangedEvent& PropertyChangedEvent)
{
	Super::PostEditChangeProperty(PropertyChangedEvent);
}

void ASVONavTestActor::PostEditUndo()
{
	Super::PostEditUndo();
}

#endif

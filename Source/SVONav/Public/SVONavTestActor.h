// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"

#include "SVONavComponent.h"
#include "Components/DrawSphereComponent.h"
#include "GameFramework/Actor.h"
#include "SVONavTestActor.generated.h"

UCLASS()
class SVONAV_API ASVONavTestActor : public AActor
{
	GENERATED_BODY()

public:
	// Sets default values for this actor's properties
	ASVONavTestActor();

	UPROPERTY(EditAnywhere, Category=SVONav)
	ASVONavTestActor* SVONavTestActor;

	UPROPERTY(EditAnywhere, Category=SVONav)
	bool IsPathStart;

	UPROPERTY(EditAnywhere, Category=SVONav)
    class USphereComponent* SphereComp;
    
    UPROPERTY(EditAnywhere, Category=SVONav)
    class USVONavComponent* NavComponent;

#if WITH_EDITOR
	virtual void EditorApplyTranslation(const FVector& DeltaTranslation, bool bAltDown, bool bShiftDown, bool bCtrlDown) override;
	virtual void PostEditChangeProperty(struct FPropertyChangedEvent& PropertyChangedEvent) override;
	virtual void PostEditUndo() override;
#endif 
	
protected:

	FThreadSafeBool ThreadSafe = true;
	
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	bool FlushDebugLine();
public:
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	bool InitStartAndGoalActor();

	void FindPath();
};


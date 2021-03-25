#pragma once

#include <Runtime/AIModule/Classes/AITypes.h>

typedef uint_fast64_t mortoncode_t;

UENUM(BlueprintType)
namespace ESVONavPathfindingRequestResult
{
	enum Type
	{
		Failed,		   // Something went wrong
        ReadyToPath,   // Pre-reqs satisfied
        AlreadyAtGoal, // No need to move
        Deferred,	   // Passed request to another thread, need to wait
        Success		   // it worked!
    };
}

struct SVONAV_API FSVONavPathfindingRequestResult
{
	FAIRequestID MoveId;
	TEnumAsByte<ESVONavPathfindingRequestResult::Type> Code;

	FSVONavPathfindingRequestResult()
        : MoveId(FAIRequestID::InvalidRequest)
        , Code(ESVONavPathfindingRequestResult::Failed)
	{
	}
	operator ESVONavPathfindingRequestResult::Type() const { return Code; }
};

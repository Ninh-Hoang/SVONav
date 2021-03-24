#pragma once

#include "CoreMinimal.h"
#include "UnrealEd.h"

DECLARE_LOG_CATEGORY_EXTERN(LogSVONavEditor, Log, All)

class FSVONavEditorModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;

};
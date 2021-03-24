#pragma once

#if WITH_EDITOR
DECLARE_LOG_CATEGORY_EXTERN(LogSVONav, Log, All);
#endif

class FSVONavModule : public IModuleInterface
{
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};

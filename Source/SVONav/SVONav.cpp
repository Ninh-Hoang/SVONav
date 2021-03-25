#include "SVONav.h"

#if WITH_EDITOR
DEFINE_LOG_CATEGORY(LogSVONav)
DEFINE_LOG_CATEGORY(VLogSVONav)
#endif

#define LOCTEXT_NAMESPACE "FSVONavModule"

void FSVONavModule::StartupModule()
{
	UE_LOG(LogSVONav, Warning, TEXT("SVONav: Module Startup"));
}

void FSVONavModule::ShutdownModule()
{
	UE_LOG(LogSVONav, Warning, TEXT("SVONav: Module Shutdown"));
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FSVONavModule, SVONav)
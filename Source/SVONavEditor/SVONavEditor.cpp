#include "SVONavEditor/SVONavEditor.h"
#include "SVONavVolumeProperties.h"
#include "SVONavVolumeBaseProperties.h"
#include "PropertyEditor/Public/PropertyEditorModule.h"

IMPLEMENT_GAME_MODULE(FSVONavEditorModule, SVONavEditor);
DEFINE_LOG_CATEGORY(LogSVONavEditor)
#define LOCTEXT_NAMESPACE "SVONavEditor"

void FSVONavEditorModule::StartupModule()
{
	UE_LOG(LogSVONavEditor, Warning, TEXT("SVONavEditor: Module Startup"));
	FPropertyEditorModule& PropertyModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
	PropertyModule.RegisterCustomClassLayout("SVONavVolume", FOnGetDetailCustomizationInstance::CreateStatic(&FSVONavVolumeProperties::MakeInstance));
	PropertyModule.RegisterCustomClassLayout("SVONavVolumeBase", FOnGetDetailCustomizationInstance::CreateStatic(&FSVONavVolumeBaseProperties::MakeInstance));
}

void FSVONavEditorModule::ShutdownModule()
{
	UE_LOG(LogSVONavEditor, Warning, TEXT("SVONavEditor: Module Shutdown"));
	FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
}

#undef LOCTEXT_NAMESPACE
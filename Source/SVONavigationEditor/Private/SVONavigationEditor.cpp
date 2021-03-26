#include "SVONavigationEditor/SVONavigationEditor.h"
#include "SVONavigationVolumeProperties.h"

#include <PropertyEditor/Public/PropertyEditorModule.h>

IMPLEMENT_GAME_MODULE(FSVONavigationEditorModule, SVONavigationEditor);

DEFINE_LOG_CATEGORY(LogSVONavigationEditor)

#define LOCTEXT_NAMESPACE "SVONavigationEditor"

void FSVONavigationEditorModule::StartupModule()
{
	UE_LOG(LogSVONavigationEditor, Warning, TEXT("SVONavigationEditor: Module Startup"));

	FPropertyEditorModule& PropertyModule = FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
	PropertyModule.RegisterCustomClassLayout("SVONavigationVolume", FOnGetDetailCustomizationInstance::CreateStatic(&FSVONavigationVolumeProperties::MakeInstance));
}

void FSVONavigationEditorModule::ShutdownModule()
{
	UE_LOG(LogSVONavigationEditor, Warning, TEXT("SVONavigationEditor: Module Shutdown"));
	FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");
}

#undef LOCTEXT_NAMESPACE
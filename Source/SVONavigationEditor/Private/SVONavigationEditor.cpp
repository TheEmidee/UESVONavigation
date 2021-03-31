#include "SVONavigationEditor/SVONavigationEditor.h"

#include <PropertyEditor/Public/PropertyEditorModule.h>

IMPLEMENT_GAME_MODULE(FSVONavigationEditorModule, SVONavigationEditor);

DEFINE_LOG_CATEGORY(LogSVONavigationEditor)

#define LOCTEXT_NAMESPACE "SVONavigationEditor"

void FSVONavigationEditorModule::StartupModule()
{
	UE_LOG(LogSVONavigationEditor, Warning, TEXT("SVONavigationEditor: Module Startup"));
}

void FSVONavigationEditorModule::ShutdownModule()
{
	UE_LOG(LogSVONavigationEditor, Warning, TEXT("SVONavigationEditor: Module Shutdown"));
}

#undef LOCTEXT_NAMESPACE
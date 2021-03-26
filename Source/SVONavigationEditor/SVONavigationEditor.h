#pragma once

#include <CoreMinimal.h>
#include <UnrealEd.h>

DECLARE_LOG_CATEGORY_EXTERN(LogSVONavigationEditor, Log, All)

class FSVONavigationEditorModule : public IModuleInterface
{
public:
	void StartupModule() override;
	void ShutdownModule() override;
};
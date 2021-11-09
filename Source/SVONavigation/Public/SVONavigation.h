#pragma once

#include <CoreMinimal.h>

class FSVONavigationModule final : public IModuleInterface
{
public:

	void StartupModule() override;
	void ShutdownModule() override;
};

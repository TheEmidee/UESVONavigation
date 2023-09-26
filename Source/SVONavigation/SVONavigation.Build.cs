// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class SVONavigation : ModuleRules
{
    public SVONavigation(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
        bUseUnity = true;
        

        PublicIncludePaths.AddRange(
            new string[] {
				// ... add public include paths required here ...
			}
            );


        PrivateIncludePaths.AddRange(
            new string[] {
                "SVONavigation/Private",
				// ... add other private include paths required here ...
			}
            );


        PublicDependencyModuleNames.AddRange(
            new string[]
            {
                "Core",
				// ... add other public dependencies that you statically link with here ...
			}
            );


        PrivateDependencyModuleNames.AddRange(
            new string[]
            {
                "CoreUObject",
                "Engine",
                "Slate",
                "SlateCore",
                "RHI",
                "RenderCore",
                "DeveloperSettings",
                "GameplayTasks",
                "AIModule",
                "NavigationSystem",
            }
            );


        DynamicallyLoadedModuleNames.AddRange(
            new string[]
            {
				// ... add any modules that your module loads dynamically here ...
			}
            );

        if (Target.bBuildEditor == true)
        {
            // @todo api: Only public because of WITH_EDITOR and UNREALED_API
            PublicDependencyModuleNames.Add("UnrealEd");
        }
    }
}

using UnrealBuildTool;

public class SVONavigationEditor : ModuleRules
{
	public SVONavigationEditor(ReadOnlyTargetRules Target) : base(Target) {

	    PublicDependencyModuleNames.AddRange( new string[] { 
			"Core", 
			"CoreUObject", 
			"Engine",  
			"SVONavigation", 
			"InputCore"
		});

	    PrivateDependencyModuleNames.AddRange( new string[] { 
			"Slate", 
			"SlateCore", 
			"PropertyEditor", 
			"EditorStyle", 
			"UnrealEd", 
			"GraphEditor", 
			"BlueprintGraph" 
		});

	    PrivateIncludePaths.AddRange( new string[] { 
			"SVONavigationEditor/Private"
		} );

	    PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
    }
};
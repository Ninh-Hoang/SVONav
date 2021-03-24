using UnrealBuildTool;

public class SVONavEditor : ModuleRules
{
	public SVONavEditor(ReadOnlyTargetRules Target) : base(Target) {

	    PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine",  "SVONav", "InputCore"});
	    PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore", "PropertyEditor", "EditorStyle", "UnrealEd", "GraphEditor", "BlueprintGraph" });
	    PrivateIncludePaths.AddRange(new string[] { "SVONavEditor/Private"} );
	    PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

    }
};
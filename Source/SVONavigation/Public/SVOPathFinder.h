#pragma once

struct FSVOPathFinderDebugInfos;
class FSVOPathFindingAlgorithmStepper;
struct FPathFindingQuery;
class ASVONavigationData;

class SVONAVIGATION_API FSVOPathFinder
{
public:
    static ENavigationQueryResult::Type GetPath( FNavigationPath & navigation_path, const FNavAgentProperties & agent_properties, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query );
    static TSharedPtr< FSVOPathFindingAlgorithmStepper > GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FNavAgentProperties & agent_properties, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query );
};

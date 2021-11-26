#pragma once

struct FSVONavigationPath;
struct FSVOPathFinderDebugInfos;
class FSVOPathFindingAlgorithmStepper;
struct FPathFindingQuery;
class ASVONavigationData;

class SVONAVIGATION_API FSVOPathFinder
{
public:
    static ENavigationQueryResult::Type GetPath( FSVONavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FSharedConstNavQueryFilter & nav_query_filter );
    static TSharedPtr< FSVOPathFindingAlgorithmStepper > GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FSharedConstNavQueryFilter & nav_query_filter );
};

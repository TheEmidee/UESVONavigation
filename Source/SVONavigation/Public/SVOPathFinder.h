#pragma once

class ASVONavigationData;
struct FPathFindingQuery;

class SVONAVIGATION_API FSVOPathFinder
{
public:
    static ENavigationQueryResult::Type GetPath( FNavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query );
};

#pragma once

#include "SVONavigationPath.h"
#include <AI/Navigation/NavigationTypes.h>

class ASVONavigationData;
struct FPathFindingQuery;

class FSVONavigationPathFinderImpl
{
public:
    static FPathFindingResult FindPath( const FNavAgentProperties & agent_properties, const FPathFindingQuery & path_finding_query );
};
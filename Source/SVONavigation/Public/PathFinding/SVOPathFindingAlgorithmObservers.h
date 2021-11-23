#pragma once

#include "SVONavigationTypes.h"

#include <GraphAStar.h>

struct FSVONavigationPath;
class FSVOPathFindingAlgorithmStepper;
struct FSVOPathFinderDebugInfos;

class FSVOPathFindingAlgorithmObserver
{
public:
    explicit FSVOPathFindingAlgorithmObserver( const FSVOPathFindingAlgorithmStepper & stepper );

    virtual ~FSVOPathFindingAlgorithmObserver() = default;

    virtual void OnProcessSingleNode( const FGraphAStarDefaultNode< FSVOVolumeNavigationData > & node )
    {}
    virtual void OnProcessNeighbor( const FGraphAStarDefaultNode< FSVOVolumeNavigationData > & parent, const FGraphAStarDefaultNode< FSVOVolumeNavigationData > & neighbor, const float cost )
    {}
    virtual void OnProcessNeighbor( const FGraphAStarDefaultNode< FSVOVolumeNavigationData > & neighbor )
    {}
    virtual void OnSearchSuccess( const TArray< FSVONodeAddress > & node_addresses )
    {}

protected:
    const FSVOPathFindingAlgorithmStepper & Stepper;
};

class FSVOPathFindingAStarObserver_BuildPath final : public FSVOPathFindingAlgorithmObserver
{
public:
    FSVOPathFindingAStarObserver_BuildPath( FSVONavigationPath & navigation_path, const FSVOPathFindingAlgorithmStepper & stepper );

    void OnSearchSuccess( const TArray< FSVONodeAddress > & ) override;

private:
    FSVONavigationPath & NavigationPath;
};

class FSVOPathFindingAStarObserver_GenerateDebugInfos final : public FSVOPathFindingAlgorithmObserver
{
public:
    FSVOPathFindingAStarObserver_GenerateDebugInfos( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingAlgorithmStepper & stepper );

    void OnProcessSingleNode( const FGraphAStarDefaultNode< FSVOVolumeNavigationData > & node ) override;
    void OnProcessNeighbor( const FGraphAStarDefaultNode< FSVOVolumeNavigationData > & parent, const FGraphAStarDefaultNode< FSVOVolumeNavigationData > & neighbor, const float cost ) override;
    void OnProcessNeighbor( const FGraphAStarDefaultNode< FSVOVolumeNavigationData > & neighbor ) override;
    void OnSearchSuccess( const TArray< FSVONodeAddress > & ) override;

private:
    void FillCurrentBestPath( const TArray< FSVONodeAddress > & node_addresses, bool add_end_location ) const;
    FSVOPathFinderDebugInfos & DebugInfos;
};
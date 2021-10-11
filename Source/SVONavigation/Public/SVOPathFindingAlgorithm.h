#pragma once

#include "SVONavigationTypes.h"

#include <NavigationData.h>

#include "SVOPathFindingAlgorithm.generated.h"

class FSVOBoundsNavigationData;
class USVOPathCostCalculator;
class USVOPathHeuristicCalculator;
class FSVONavigationQueryFilterImpl;
class ASVONavigationData;

struct FSVOLinkWithCost
{
    FSVOLinkWithCost( const FSVOOctreeLink & link, float cost ) :
        Link( link ),
        Cost( cost )
    {
    }

    FSVOOctreeLink Link;
    float Cost;

    bool operator<( const FSVOLinkWithCost & other ) const
    {
        return Cost > other.Cost;
    }
};

struct FSVOPathFinderDebugCost
{
    void Reset();
    
    FVector Location = { 0.0f, 0.0f, 0.0f };
    float Cost = -1.0f;
    bool WasEvaluated = false;
};

struct FSVOPathFinderDebugStep
{
    void Reset();
    
    FSVOPathFinderDebugCost CurrentLocationCost;
    TArray< FSVOPathFinderDebugCost, TInlineAllocator< 6 > > NeighborLocationCosts;
};

USTRUCT()
struct SVONAVIGATION_API FSVOPathFinderDebugInfos
{
    GENERATED_USTRUCT_BODY()

    void Reset();

    TArray< FSVOPathFinderDebugStep > DebugSteps;
    FNavigationPath CurrentBestPath;

    UPROPERTY( VisibleAnywhere )
    int Iterations;

    UPROPERTY( VisibleAnywhere )
    int VisitedNodes;
};

struct FSVOPathFindingParameters
{
    FSVOPathFindingParameters( const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query );

    const ASVONavigationData & NavigationData;
    FVector StartLocation;
    FVector EndLocation;
    const FNavigationQueryFilter & NavigationQueryFilter;
    const FSVONavigationQueryFilterImpl * QueryFilterImplementation;
    const FSVONavigationQueryFilterSettings & QueryFilterSettings;
    const USVOPathHeuristicCalculator * HeuristicCalculator;
    const USVOPathCostCalculator * CostCalculator;
    const FSVOBoundsNavigationData * BoundsNavigationData;
    FSVOOctreeLink StartLink;
    FSVOOctreeLink EndLink;
    float VerticalOffset;
};

class FSVOPathFindingAlgorithmObserver
{
public:
    FSVOPathFindingAlgorithmObserver() = default;

    virtual ~FSVOPathFindingAlgorithmObserver() = default;
    virtual void OnOpenNode( const FSVOLinkWithCost & link_with_cost ) {}
    virtual void OnNeighborCostComputed( const FSVOOctreeLink & link, float cost ) {}
    virtual void OnFoundBetterNeighbor() {}
    virtual void OnNeighborVisited() {}
    virtual void OnEndLinkReached() {}
    virtual void OnEndStep() {}
};

template< typename _ALGO_ >
class TSVOPathFindingAlgorithmObserver : public FSVOPathFindingAlgorithmObserver
{
public:
    explicit TSVOPathFindingAlgorithmObserver( _ALGO_ & algo );

protected:

    _ALGO_ & Algo;
};

class FSVOPathFindingAlgorithmStepper
{
public:
    explicit FSVOPathFindingAlgorithmStepper( const FSVOPathFindingParameters & params );

    const FSVOPathFindingParameters & GetParams() const;
    
    virtual ~FSVOPathFindingAlgorithmStepper() = default;
    virtual bool Step( ENavigationQueryResult::Type & result ) = 0;

    void AddObserver( TSharedPtr< FSVOPathFindingAlgorithmObserver > observer );

protected:

    FSVOPathFindingParameters Params;
    TArray< TSharedPtr< FSVOPathFindingAlgorithmObserver > > Observers;
};

FORCEINLINE const FSVOPathFindingParameters & FSVOPathFindingAlgorithmStepper::GetParams() const
{
    return Params;
}

class FSVOPathFindingAlgorithmStepper_AStar : public FSVOPathFindingAlgorithmStepper
{
public:
    explicit FSVOPathFindingAlgorithmStepper_AStar( const FSVOPathFindingParameters & params );

    const TMap< FSVOOctreeLink, FSVOOctreeLink > & GetCameFrom() const;
    bool Step( ENavigationQueryResult::Type & result ) override;

private:

    TArray< FSVOLinkWithCost > Frontier;
    TMap< FSVOOctreeLink, FSVOOctreeLink > CameFrom;
    TMap< FSVOOctreeLink, float > CostSoFar;
};

FORCEINLINE const TMap< FSVOOctreeLink, FSVOOctreeLink > & FSVOPathFindingAlgorithmStepper_AStar::GetCameFrom() const
{
    return CameFrom;   
}

class FSVOPathFindingAStarObserver_BuildPath : public TSVOPathFindingAlgorithmObserver< FSVOPathFindingAlgorithmStepper_AStar >
{
public:
    FSVOPathFindingAStarObserver_BuildPath( FNavigationPath & navigation_path, FSVOPathFindingAlgorithmStepper_AStar & algo );

    void OnEndLinkReached() override;

private:
    FNavigationPath & NavigationPath;
};

class FSVOPathFindingAStarObserver_GenerateDebugInfos : public TSVOPathFindingAlgorithmObserver< FSVOPathFindingAlgorithmStepper_AStar >
{
public:
    FSVOPathFindingAStarObserver_GenerateDebugInfos( FSVOPathFinderDebugInfos & debug_infos, FSVOPathFindingAlgorithmStepper_AStar & algo );

    void OnOpenNode( const FSVOLinkWithCost & link_with_cost ) override;
    void OnNeighborCostComputed( const FSVOOctreeLink & link, float cost ) override;
    void OnEndLinkReached() override;
    void OnNeighborVisited() override;
    void OnFoundBetterNeighbor() override;
    void OnEndStep() override;
    
private:
    FSVOPathFinderDebugInfos & DebugInfos;
    FSVOPathFinderDebugStep DebugStep;
    FSVOOctreeLink CurrentLink;
    FSVOPathFinderDebugCost CurrentNeighborDebugCost;
};

UCLASS( HideDropdown, NotBlueprintable )
class SVONAVIGATION_API USVOPathFindingAlgorithm : public UObject
{
    GENERATED_BODY()

public:
    virtual ENavigationQueryResult::Type GetPath( FNavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const;
    virtual TSharedPtr< FSVOPathFindingAlgorithmStepper > GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const;
};

UCLASS()
class SVONAVIGATION_API USVOPathFindingAlgorithmAStar final : public USVOPathFindingAlgorithm
{
    GENERATED_BODY()

public:
    ENavigationQueryResult::Type GetPath( FNavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const override;    
    TSharedPtr< FSVOPathFindingAlgorithmStepper > GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const override;
};
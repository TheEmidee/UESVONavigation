#pragma once

#include "Chaos/AABB.h"
#include "GraphAStar.h"
#include "SVONavigationTypes.h"

#include <NavigationData.h>

#include "SVOPathFindingAlgorithm.generated.h"

struct FSVOLinkWithCost;
class FSVOBoundsNavigationData;
class USVOPathCostCalculator;
class USVOPathHeuristicCalculator;
class FSVONavigationQueryFilterImpl;
class ASVONavigationData;

struct FSVOLinkWithLocation
{
    FSVOLinkWithLocation() = default;
    FSVOLinkWithLocation( const FSVOOctreeLink & link, const FVector & location );
    FSVOLinkWithLocation( const FSVOOctreeLink & link, const FSVOBoundsNavigationData & bounds_navigation_data );

    bool operator==( const FSVOLinkWithLocation & other ) const
    {
        return Link == other.Link;
    }

    bool operator!=( const FSVOLinkWithLocation & other ) const
    {
        return !operator==( other );
    }

    FSVOOctreeLink Link;
    FVector Location;
};

FORCEINLINE uint32 GetTypeHash( const FSVOLinkWithLocation & link_with_location )
{
    return GetTypeHash( link_with_location.Link );
}

struct FSVOLinkWithCost
{
    FSVOLinkWithCost( const FSVOLinkWithLocation & link_with_location, const float cost ) :
        LinkWithLocation( link_with_location ),
        Cost( cost )
    {
    }

    FSVOLinkWithLocation LinkWithLocation;
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
    FSVOPathFindingParameters( const FNavAgentProperties & agent_properties, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query );

    const FNavAgentProperties & AgentProperties;
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

enum class ESVOPathFindingAlgorithmState : uint8
{
    Init,
    ProcessNode,
    ProcessNeighbor,
    Ended
};

enum class ESVOPathFindingAlgorithmStepperStatus : uint8
{
    MustContinue,
    IsStopped
};

class FSVOPathFindingAlgorithmStepper;

class FSVOPathFindingAlgorithmObserver
{
public:
    explicit FSVOPathFindingAlgorithmObserver( const FSVOPathFindingAlgorithmStepper & stepper );

    virtual ~FSVOPathFindingAlgorithmObserver() = default;

    virtual void OnOpenNode( const FSVOLinkWithCost & link_with_cost )
    {}
    virtual void OnNeighborCostComputed( const FSVOOctreeLink & link, float cost )
    {}
    virtual void OnFoundBetterNeighbor()
    {}
    virtual void OnNeighborVisited()
    {}
    virtual void OnEndLinkReached()
    {}
    virtual void OnSearchSuccess( const TArray< FSVOOctreeLink > & link_path )
    {}

protected:
    const FSVOPathFindingAlgorithmStepper & Stepper;
};

class FSVOPathFindingAlgorithmStepper : protected FGraphAStar< FSVOBoundsNavigationData, FGraphAStarDefaultPolicy, FGraphAStarDefaultNode< FSVOBoundsNavigationData > >
{
public:
    explicit FSVOPathFindingAlgorithmStepper( const FSVOPathFindingParameters & parameters );

    const FSVOPathFindingParameters & GetParameters() const;
    ESVOPathFindingAlgorithmStepperStatus Step( EGraphAStarResult & result );
    void AddObserver( TSharedPtr< FSVOPathFindingAlgorithmObserver > observer );

protected:
    ESVOPathFindingAlgorithmStepperStatus Init( EGraphAStarResult & result );
    ESVOPathFindingAlgorithmStepperStatus ProcessSingleNode( EGraphAStarResult & result );
    ESVOPathFindingAlgorithmStepperStatus ProcessNeighbor();
    ESVOPathFindingAlgorithmStepperStatus Ended( EGraphAStarResult & result );

    void SetState( ESVOPathFindingAlgorithmState new_state );

private:
    ESVOPathFindingAlgorithmState State;
    const FSVOPathFindingParameters & Parameters;
    int32 ConsideredNodeIndex;
    int32 BestNodeIndex;
    float BestNodeCost;
    TArray< FSVOOctreeLink > Neighbors;
    int NeighborIndex;
    TArray< TSharedPtr< FSVOPathFindingAlgorithmObserver > > Observers;
};

FORCEINLINE const FSVOPathFindingParameters & FSVOPathFindingAlgorithmStepper::GetParameters() const
{
    return Parameters;
}

//class FSVOPathFindingAlgorithmStepper
//{
//public:
//    explicit FSVOPathFindingAlgorithmStepper( const FSVOPathFindingParameters & params );
//
//    const FSVOPathFindingParameters & GetParams() const;
//
//    virtual ~FSVOPathFindingAlgorithmStepper() = default;
//    virtual bool Step( ENavigationQueryResult::Type & result ) = 0;
//
//    void AddObserver( TSharedPtr< FSVOPathFindingAlgorithmObserver > observer );
//
//protected:
//    FSVOPathFindingParameters Params;
//    TArray< TSharedPtr< FSVOPathFindingAlgorithmObserver > > Observers;
//};
//
//FORCEINLINE const FSVOPathFindingParameters & FSVOPathFindingAlgorithmStepper::GetParams() const
//{
//    return Params;
//}
//
//class FSVOPathFindingAlgorithmStepper_AStar : public FSVOPathFindingAlgorithmStepper
//{
//public:
//    explicit FSVOPathFindingAlgorithmStepper_AStar( const FSVOPathFindingParameters & params );
//
//    const TMap< FSVOLinkWithLocation, FSVOLinkWithLocation > & GetCameFrom() const;
//    bool Step( ENavigationQueryResult::Type & result ) override;
//
//protected:
//    virtual void ProcessNeighbor( FSVOLinkWithLocation current, FSVOOctreeLink neighbor );
//
//    TArray< FSVOLinkWithCost > OpenSet;
//    TMap< FSVOLinkWithLocation, FSVOLinkWithLocation > CameFrom;
//    TMap< FSVOOctreeLink, float > CostSoFar;
//};
//
//FORCEINLINE const TMap< FSVOLinkWithLocation, FSVOLinkWithLocation > & FSVOPathFindingAlgorithmStepper_AStar::GetCameFrom() const
//{
//    return CameFrom;
//}
//
//USTRUCT()
//struct SVONAVIGATION_API FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters
//{
//    GENERATED_USTRUCT_BODY()
//
//    FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters() :
//        AgentRadiusMultiplier( 1.0f ),
//        bShowLineOfSightTraces( false ),
//        TraceType( ETraceTypeQuery::TraceTypeQuery1 )
//    {}
//
//    UPROPERTY( EditAnywhere )
//    float AgentRadiusMultiplier;
//
//    UPROPERTY( EditAnywhere )
//    uint8 bShowLineOfSightTraces : 1;
//
//    UPROPERTY( EditAnywhere )
//    TEnumAsByte< ETraceTypeQuery > TraceType;
//};
//
//class FSVOPathFindingAlgorithmStepper_ThetaStar final : public FSVOPathFindingAlgorithmStepper_AStar
//{
//public:
//    FSVOPathFindingAlgorithmStepper_ThetaStar( const FSVOPathFindingParameters & params, const FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters & theta_params );
//
//private:
//    void ProcessNeighbor( FSVOLinkWithLocation current, FSVOOctreeLink neighbor ) override;
//    bool IsInLineOfSight( const FSVOLinkWithLocation & from, FSVOOctreeLink to ) const;
//
//    FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters ThetaStarParams;
//};

class FSVOPathFindingAStarObserver_BuildPath final : public FSVOPathFindingAlgorithmObserver
{
public:
    FSVOPathFindingAStarObserver_BuildPath( FNavigationPath & navigation_path, const FSVOPathFindingAlgorithmStepper & stepper );

    void OnSearchSuccess( const TArray< FSVOOctreeLink > & ) override;

private:
    FNavigationPath & NavigationPath;
};

class FSVOPathFindingAStarObserver_GenerateDebugInfos // : public TSVOPathFindingAlgorithmObserver< FSVOPathFindingAlgorithmStepper_AStar >
{
public:
    FSVOPathFindingAStarObserver_GenerateDebugInfos( FSVOPathFinderDebugInfos & debug_infos /*, FSVOPathFindingAlgorithmStepper_AStar & algo*/ );

    /*void OnOpenNode( const FSVOLinkWithCost & link_with_cost ) override;
    void OnNeighborCostComputed( const FSVOOctreeLink & link, float cost ) override;
    void OnEndLinkReached() override;
    void OnNeighborVisited() override;
    void OnFoundBetterNeighbor() override;
    void OnEndStep() override;*/

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

//UCLASS( Blueprintable )
//class SVONAVIGATION_API USVOPathFindingAlgorithmThetaStar final : public USVOPathFindingAlgorithm
//{
//    GENERATED_BODY()
//
//public:
//    ENavigationQueryResult::Type GetPath( FNavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const override;
//    TSharedPtr< FSVOPathFindingAlgorithmStepper > GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const override;
//
//private:
//    UPROPERTY( EditAnywhere )
//    FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters Parameters;
//};
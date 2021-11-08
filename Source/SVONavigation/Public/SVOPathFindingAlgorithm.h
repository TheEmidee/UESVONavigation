#pragma once

#include "SVONavigationTypes.h"

#include <GraphAStar.h>
#include <NavigationData.h>

#include "SVOPathFindingAlgorithm.generated.h"

struct FSVOLinkWithCost;
class FSVOBoundsNavigationData;
class USVOTraversalCostCalculator;
class USVOHeuristicCalculator;
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

struct FSVOPathFinderDebugNodeCost
{
    FSVOPathFinderDebugNodeCost() = default;
    FSVOPathFinderDebugNodeCost( const FSVOLinkWithLocation & from, const FSVOLinkWithLocation & to, const float cost, const bool is_closed );

    void Reset();

    FSVOLinkWithLocation From;
    FSVOLinkWithLocation To;
    float Cost;
    uint8 bIsClosed : 1;
};

USTRUCT()
struct SVONAVIGATION_API FSVOPathFinderDebugInfos
{
    GENERATED_USTRUCT_BODY()

    void Reset();

    FSVOPathFinderDebugNodeCost LastProcessedSingleNode;
    TArray< FSVOPathFinderDebugNodeCost > ProcessedNeighbors;
    FNavigationPath CurrentBestPath;

    UPROPERTY( VisibleAnywhere )
    int Iterations;

    UPROPERTY( VisibleAnywhere )
    int VisitedNodes;

    UPROPERTY( VisibleAnywhere )
    int PathSegmentCount;

    UPROPERTY( VisibleAnywhere )
    float PathLength;
};

struct FSVOPathFindingParameters
{
    FSVOPathFindingParameters( const FNavAgentProperties & agent_properties, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query );

    // By copy because it can be constructed as a temp variable
    const FNavAgentProperties AgentProperties;
    const ASVONavigationData & NavigationData;
    FVector StartLocation;
    FVector EndLocation;
    const FNavigationQueryFilter & NavigationQueryFilter;
    const FSVONavigationQueryFilterImpl * QueryFilterImplementation;
    const FSVONavigationQueryFilterSettings & QueryFilterSettings;
    const USVOHeuristicCalculator * HeuristicCalculator;
    const USVOTraversalCostCalculator * CostCalculator;
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

    virtual void OnProcessSingleNode( const FGraphAStarDefaultNode< FSVOBoundsNavigationData > & node )
    {}
    virtual void OnProcessNeighbor( const FGraphAStarDefaultNode< FSVOBoundsNavigationData > & parent, const FGraphAStarDefaultNode< FSVOBoundsNavigationData > & neighbor, const float cost )
    {}
    virtual void OnProcessNeighbor( const FGraphAStarDefaultNode< FSVOBoundsNavigationData > & neighbor )
    {}
    virtual void OnSearchSuccess( const TArray< FSVOOctreeLink > & link_path )
    {}

protected:
    const FSVOPathFindingAlgorithmStepper & Stepper;
};

class FSVOPathFindingAStarObserver_BuildPath final : public FSVOPathFindingAlgorithmObserver
{
public:
    FSVOPathFindingAStarObserver_BuildPath( FNavigationPath & navigation_path, const FSVOPathFindingAlgorithmStepper & stepper );

    void OnSearchSuccess( const TArray< FSVOOctreeLink > & ) override;

private:
    FNavigationPath & NavigationPath;
};

class FSVOPathFindingAStarObserver_GenerateDebugInfos final : public FSVOPathFindingAlgorithmObserver
{
public:
    FSVOPathFindingAStarObserver_GenerateDebugInfos( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingAlgorithmStepper & stepper );

    void OnProcessSingleNode( const FGraphAStarDefaultNode< FSVOBoundsNavigationData > & node ) override;
    void OnProcessNeighbor( const FGraphAStarDefaultNode< FSVOBoundsNavigationData > & parent, const FGraphAStarDefaultNode< FSVOBoundsNavigationData > & neighbor, const float cost ) override;
    void OnProcessNeighbor( const FGraphAStarDefaultNode< FSVOBoundsNavigationData > & neighbor ) override;
    void OnSearchSuccess( const TArray< FSVOOctreeLink > & ) override;

private:
    void FillCurrentBestPath( const TArray< FSVOOctreeLink > & link_path, bool add_end_location ) const;
    FSVOPathFinderDebugInfos & DebugInfos;
};

class FSVOGraphAStar final : public FGraphAStar< FSVOBoundsNavigationData, FGraphAStarDefaultPolicy, FGraphAStarDefaultNode< FSVOBoundsNavigationData > >
{
public:
    explicit FSVOGraphAStar( const FSVOBoundsNavigationData & graph );
};

// This class is a wrapper around FGraphAStar.
// Internally it's just a state machine which calls one of the pure virtual functions in Step, until that function returns ESVOPathFindingAlgorithmStepperStatus::IsStopped
// It accepts observers to do something while the path is being generated (for debug purposes for example) or when the path finding ends (to construct the navigation path)
class FSVOPathFindingAlgorithmStepper
{
public:
    explicit FSVOPathFindingAlgorithmStepper( const FSVOPathFindingParameters & parameters );
    virtual ~FSVOPathFindingAlgorithmStepper() = default;

    ESVOPathFindingAlgorithmState GetState() const;
    const FSVOPathFindingParameters & GetParameters() const;
    void AddObserver( TSharedPtr< FSVOPathFindingAlgorithmObserver > observer );
    const FSVOGraphAStar & GetGraph() const;

    ESVOPathFindingAlgorithmStepperStatus Step( EGraphAStarResult & result );
    virtual bool FillLinkPath( TArray< FSVOOctreeLink > & link_path ) const;

protected:
    virtual ESVOPathFindingAlgorithmStepperStatus Init( EGraphAStarResult & result ) = 0;
    virtual ESVOPathFindingAlgorithmStepperStatus ProcessSingleNode( EGraphAStarResult & result ) = 0;
    virtual ESVOPathFindingAlgorithmStepperStatus ProcessNeighbor() = 0;
    virtual ESVOPathFindingAlgorithmStepperStatus Ended( EGraphAStarResult & result ) = 0;

    void SetState( ESVOPathFindingAlgorithmState new_state );
    float GetHeuristicCost( const FSVOOctreeLink & from, const FSVOOctreeLink & to ) const;
    float GetTraversalCost( const FSVOOctreeLink & from, const FSVOOctreeLink & to ) const;

    FSVOGraphAStar Graph;
    ESVOPathFindingAlgorithmState State;
    FSVOPathFindingParameters Parameters;
    TArray< TSharedPtr< FSVOPathFindingAlgorithmObserver > > Observers;
};

FORCEINLINE ESVOPathFindingAlgorithmState FSVOPathFindingAlgorithmStepper::GetState() const
{
    return State;
}

FORCEINLINE const FSVOPathFindingParameters & FSVOPathFindingAlgorithmStepper::GetParameters() const
{
    return Parameters;
}

FORCEINLINE const FSVOGraphAStar & FSVOPathFindingAlgorithmStepper::GetGraph() const
{
    return Graph;
}

class FSVOPathFindingAlgorithmStepper_AStar : public FSVOPathFindingAlgorithmStepper
{
public:
    explicit FSVOPathFindingAlgorithmStepper_AStar( const FSVOPathFindingParameters & parameters );

    bool FillLinkPath( TArray< FSVOOctreeLink > & link_path ) const override;

protected:
    ESVOPathFindingAlgorithmStepperStatus Init( EGraphAStarResult & result ) override;
    ESVOPathFindingAlgorithmStepperStatus ProcessSingleNode( EGraphAStarResult & result ) override;
    ESVOPathFindingAlgorithmStepperStatus ProcessNeighbor() override;
    ESVOPathFindingAlgorithmStepperStatus Ended( EGraphAStarResult & result ) override;

    void FillLinkNeighbors( const FSVOOctreeLink & link );
    float AdjustTotalCostWithNodeSizeCompensation( float total_cost, FSVOOctreeLink neighbor_link ) const;

    struct NeighborIndexIncrement
    {
        NeighborIndexIncrement( TArray< FSVOOctreeLink > & neighbors, int & neighbor_index, ESVOPathFindingAlgorithmState & state );
        ~NeighborIndexIncrement();

        TArray< FSVOOctreeLink > & Neighbors;
        int & NeighborIndex;
        ESVOPathFindingAlgorithmState & State;
    };

    int32 ConsideredNodeIndex;
    int32 BestNodeIndex;
    float BestNodeCost;
    int NeighborIndex;
    TArray< FSVOOctreeLink > Neighbors;
};

USTRUCT()
struct SVONAVIGATION_API FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters
{
    GENERATED_USTRUCT_BODY()

    FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters() :
        AgentRadiusMultiplier( 0.5f ),
        bShowLineOfSightTraces( false ),
        TraceType( ETraceTypeQuery::TraceTypeQuery1 )
    {}

    UPROPERTY( EditAnywhere )
    float AgentRadiusMultiplier;

    UPROPERTY( EditAnywhere )
    uint8 bShowLineOfSightTraces : 1;

    UPROPERTY( EditAnywhere )
    TEnumAsByte< ETraceTypeQuery > TraceType;
};

// See https://www.wikiwand.com/en/Theta* or http://idm-lab.org/bib/abstracts/papers/aaai07a.pdf
// This uses line of sight checks to try to shorten the path when exploring neighbors.
// If there's los between a neighbor and the parent of the current node, we skip the current node and link the parent to the neighbor
class FSVOPathFindingAlgorithmStepper_ThetaStar : public FSVOPathFindingAlgorithmStepper_AStar
{
public:
    FSVOPathFindingAlgorithmStepper_ThetaStar( const FSVOPathFindingParameters & parameters, const FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters & theta_star_parameters );

protected:
    ESVOPathFindingAlgorithmStepperStatus Init( EGraphAStarResult & result ) override;
    ESVOPathFindingAlgorithmStepperStatus ProcessNeighbor() override;
    ESVOPathFindingAlgorithmStepperStatus Ended( EGraphAStarResult & result ) override;
    bool HasLineOfSight( FSVOOctreeLink from, FSVOOctreeLink to );

    const FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters & ThetaStarParameters;

private:
    int LOSCheckCount;
};

// See http://idm-lab.org/bib/abstracts/papers/aaai10b.pdf
// Like Theta*, it uses line of sight checks to try to shorten the path when exploring neighbors.
// But it does much less LOS checks, as it does that test only when processing a node. Not between a node being processed and all its neighbors
class FSVOPathFindingAlgorithmStepper_LazyThetaStar final : public FSVOPathFindingAlgorithmStepper_ThetaStar
{
public:
    FSVOPathFindingAlgorithmStepper_LazyThetaStar( const FSVOPathFindingParameters & parameters, const FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters & theta_star_parameters );

protected:
    ESVOPathFindingAlgorithmStepperStatus ProcessSingleNode( EGraphAStarResult & result ) override;
    ESVOPathFindingAlgorithmStepperStatus ProcessNeighbor() override;
};

UCLASS( HideDropdown, NotBlueprintable, EditInlineNew )
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

UCLASS( Blueprintable )
class SVONAVIGATION_API USVOPathFindingAlgorithmThetaStar final : public USVOPathFindingAlgorithm
{
    GENERATED_BODY()

public:
    ENavigationQueryResult::Type GetPath( FNavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const override;
    TSharedPtr< FSVOPathFindingAlgorithmStepper > GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const override;

private:
    UPROPERTY( EditAnywhere )
    FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters ThetaStarParameters;
};

UCLASS( Blueprintable )
class SVONAVIGATION_API USVOPathFindingAlgorithmLazyThetaStar final : public USVOPathFindingAlgorithm
{
    GENERATED_BODY()

public:
    ENavigationQueryResult::Type GetPath( FNavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const override;
    TSharedPtr< FSVOPathFindingAlgorithmStepper > GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const override;

private:
    UPROPERTY( EditAnywhere )
    FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters ThetaStarParameters;
};

template < class _ALGO_ >
ENavigationQueryResult::Type GetPath( FNavigationPath & navigation_path, const FSVOPathFindingParameters & params, _ALGO_ * = nullptr )
{
    _ALGO_ stepper_a_star( params );
    const auto path_builder = MakeShared< FSVOPathFindingAStarObserver_BuildPath >( navigation_path, stepper_a_star );

    stepper_a_star.AddObserver( path_builder );

    int iterations = 0;

    EGraphAStarResult result = EGraphAStarResult::SearchFail;
    while ( stepper_a_star.Step( result ) == ESVOPathFindingAlgorithmStepperStatus::MustContinue )
    {
        iterations++;
    }

    return ENavigationQueryResult::Success;
}
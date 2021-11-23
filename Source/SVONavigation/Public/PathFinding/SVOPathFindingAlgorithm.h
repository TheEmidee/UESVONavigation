#pragma once

#include "SVONavigationTypes.h"
#include "SVOPathFindingAlgorithmObservers.h"
#include "SVOVolumeNavigationData.h"

#include <GraphAStar.h>
#include <NavigationPath.h>

#include "SVOPathFindingAlgorithm.generated.h"

struct FSVONavigationQueryFilterSettings;
class FSVONavigationQueryFilterImpl;
struct FPathFindingQuery;
class ASVONavigationData;

struct FSVONodeAddressWithLocation
{
    FSVONodeAddressWithLocation() = default;
    FSVONodeAddressWithLocation( const FSVONodeAddress & node_address, const FVector & location );
    FSVONodeAddressWithLocation( const FSVONodeAddress & node_address, const FSVOVolumeNavigationData & bounds_navigation_data );

    bool operator==( const FSVONodeAddressWithLocation & other ) const
    {
        return NodeAddress == other.NodeAddress;
    }

    bool operator!=( const FSVONodeAddressWithLocation & other ) const
    {
        return !operator==( other );
    }

    FSVONodeAddress NodeAddress;
    FVector Location;
};

FORCEINLINE uint32 GetTypeHash( const FSVONodeAddressWithLocation & node_address_with_location )
{
    return GetTypeHash( node_address_with_location.NodeAddress );
}

struct FSVOPathFinderDebugNodeCost
{
    FSVOPathFinderDebugNodeCost() = default;
    FSVOPathFinderDebugNodeCost( const FSVONodeAddressWithLocation & from, const FSVONodeAddressWithLocation & to, const float cost, const bool is_closed );

    void Reset();

    FSVONodeAddressWithLocation From;
    FSVONodeAddressWithLocation To;
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
    const USVOPathHeuristicCalculator * HeuristicCalculator;
    const USVOPathTraversalCostCalculator * CostCalculator;
    const FSVOVolumeNavigationData * VolumeNavigationData;
    FSVONodeAddress StartNodeAddress;
    FSVONodeAddress EndNodeAddress;
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

class FSVOGraphAStar final : public FGraphAStar< FSVOVolumeNavigationData, FGraphAStarDefaultPolicy, FGraphAStarDefaultNode< FSVOVolumeNavigationData > >
{
public:
    explicit FSVOGraphAStar( const FSVOVolumeNavigationData & graph );
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
    virtual bool FillNodeAddresses( TArray< FSVONodeAddress > & node_addresses ) const;

protected:
    virtual ESVOPathFindingAlgorithmStepperStatus Init( EGraphAStarResult & result ) = 0;
    virtual ESVOPathFindingAlgorithmStepperStatus ProcessSingleNode( EGraphAStarResult & result ) = 0;
    virtual ESVOPathFindingAlgorithmStepperStatus ProcessNeighbor() = 0;
    virtual ESVOPathFindingAlgorithmStepperStatus Ended( EGraphAStarResult & result ) = 0;

    void SetState( ESVOPathFindingAlgorithmState new_state );
    float GetHeuristicCost( const FSVONodeAddress & from, const FSVONodeAddress & to ) const;
    float GetTraversalCost( const FSVONodeAddress & from, const FSVONodeAddress & to ) const;

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

UCLASS( HideDropdown, NotBlueprintable, EditInlineNew )
class SVONAVIGATION_API USVOPathFindingAlgorithm : public UObject
{
    GENERATED_BODY()

public:
    virtual ENavigationQueryResult::Type GetPath( FNavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const;
    virtual TSharedPtr< FSVOPathFindingAlgorithmStepper > GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const;
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
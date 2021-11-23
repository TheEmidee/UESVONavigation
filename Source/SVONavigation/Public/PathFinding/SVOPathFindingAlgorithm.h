#pragma once

#include "SVOPathFindingAlgorithmObservers.h"
#include "SVOPathFindingAlgorithmTypes.h"
#include "SVOVolumeNavigationData.h"

#include "SVOPathFindingAlgorithm.generated.h"

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
    virtual bool FillNodeAddresses( TArray< FSVOPathFinderNodeAddressWithCost > & node_addresses ) const;

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
    virtual ENavigationQueryResult::Type GetPath( FSVONavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const;
    virtual TSharedPtr< FSVOPathFindingAlgorithmStepper > GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const;
};
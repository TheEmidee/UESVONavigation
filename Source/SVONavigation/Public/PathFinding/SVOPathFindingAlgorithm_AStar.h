#pragma once

#include "PathFinding/SVOPathFindingAlgorithm.h"

#include "SVOPathFindingAlgorithm_AStar.generated.h"

class FSVOPathFindingAlgorithmStepper_AStar : public FSVOPathFindingAlgorithmStepper
{
public:
    explicit FSVOPathFindingAlgorithmStepper_AStar( const FSVOPathFindingParameters & parameters );

    bool FillNodeAddresses( TArray< FSVONodeAddress > & ) const override;

protected:
    ESVOPathFindingAlgorithmStepperStatus Init( EGraphAStarResult & result ) override;
    ESVOPathFindingAlgorithmStepperStatus ProcessSingleNode( EGraphAStarResult & result ) override;
    ESVOPathFindingAlgorithmStepperStatus ProcessNeighbor() override;
    ESVOPathFindingAlgorithmStepperStatus Ended( EGraphAStarResult & result ) override;

    void FillNodeAddressNeighbors( const FSVONodeAddress & node_address );
    float AdjustTotalCostWithNodeSizeCompensation( float total_cost, FSVONodeAddress neighbor_node_address ) const;

    struct NeighborIndexIncrement
    {
        NeighborIndexIncrement( TArray< FSVONodeAddress > & neighbors, int & neighbor_index, ESVOPathFindingAlgorithmState & state );
        ~NeighborIndexIncrement();

        TArray< FSVONodeAddress > & Neighbors;
        int & NeighborIndex;
        ESVOPathFindingAlgorithmState & State;
    };

    int32 ConsideredNodeIndex;
    int32 BestNodeIndex;
    float BestNodeCost;
    int NeighborIndex;
    TArray< FSVONodeAddress > Neighbors;
};

UCLASS()
class SVONAVIGATION_API USVOPathFindingAlgorithmAStar final : public USVOPathFindingAlgorithm
{
    GENERATED_BODY()

public:
    ENavigationQueryResult::Type GetPath( FSVONavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const override;
    TSharedPtr< FSVOPathFindingAlgorithmStepper > GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const override;
};
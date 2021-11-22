#pragma once

#include "SVOPathFindingAlgorithm_AStar.h"

#include "SVOPathFindingAlgorithm_ThetaStar.generated.h"

class USVORayCaster;

USTRUCT()
struct SVONAVIGATION_API FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters
{
    GENERATED_USTRUCT_BODY()

    FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters();

    UPROPERTY( Instanced, EditAnywhere )
    USVORayCaster * RayCaster;
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
    bool HasLineOfSight( FSVONodeAddress from, FSVONodeAddress to ) const;

    const FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters & ThetaStarParameters;

private:
    int LOSCheckCount;
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
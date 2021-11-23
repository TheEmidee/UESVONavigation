#pragma once

#include "SVOPathFindingAlgorithm_ThetaStar.h"

#include "SVOPathFindingAlgorithm_LazyThetaStar.generated.h"

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

UCLASS( Blueprintable )
class SVONAVIGATION_API USVOPathFindingAlgorithmLazyThetaStar final : public USVOPathFindingAlgorithm
{
    GENERATED_BODY()

public:
    ENavigationQueryResult::Type GetPath( FSVONavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const override;
    TSharedPtr< FSVOPathFindingAlgorithmStepper > GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const override;

private:
    UPROPERTY( EditAnywhere )
    FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters ThetaStarParameters;
};
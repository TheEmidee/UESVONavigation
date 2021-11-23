#include "PathFinding/SVOPathFindingAlgorithm.h"

#include "PathFinding/SVOPathHeuristicCalculator.h"
#include "PathFinding/SVOPathTraversalCostCalculator.h"
#include "Pathfinding/SVONavigationQueryFilterImpl.h"
#include "SVONavigationData.h"
#include "SVOVolumeNavigationData.h"

FSVOGraphAStar::FSVOGraphAStar( const FSVOVolumeNavigationData & graph ) :
    FGraphAStar< FSVOVolumeNavigationData, FGraphAStarDefaultPolicy, FGraphAStarDefaultNode< FSVOVolumeNavigationData > >( graph )
{
}

FSVOPathFindingAlgorithmStepper::FSVOPathFindingAlgorithmStepper( const FSVOPathFindingParameters & parameters ) :
    Graph( parameters.VolumeNavigationData ),
    State( ESVOPathFindingAlgorithmState::Init ),
    Parameters( parameters )
{
}

void FSVOPathFindingAlgorithmStepper::AddObserver( const TSharedPtr< FSVOPathFindingAlgorithmObserver > observer )
{
    Observers.Add( observer );
}

ESVOPathFindingAlgorithmStepperStatus FSVOPathFindingAlgorithmStepper::Step( EGraphAStarResult & result )
{
    switch ( State )
    {
        case ESVOPathFindingAlgorithmState::Init:
        {
            return Init( result );
        }
        case ESVOPathFindingAlgorithmState::ProcessNode:
        {
            return ProcessSingleNode( result );
        }
        case ESVOPathFindingAlgorithmState::ProcessNeighbor:
        {
            return ProcessNeighbor();
        }
        case ESVOPathFindingAlgorithmState::Ended:
        {
            return Ended( result );
        }
        default:
        {
            checkNoEntry();
            return ESVOPathFindingAlgorithmStepperStatus::IsStopped;
        }
    }
}

bool FSVOPathFindingAlgorithmStepper::FillNodeAddresses( TArray< FSVOPathFinderNodeAddressWithCost > & node_addresses ) const
{
    checkNoEntry();
    return false;
}

void FSVOPathFindingAlgorithmStepper::SetState( const ESVOPathFindingAlgorithmState new_state )
{
    State = new_state;
}

float FSVOPathFindingAlgorithmStepper::GetHeuristicCost( const FSVONodeAddress & from, const FSVONodeAddress & to ) const
{
    return Parameters.HeuristicCalculator->GetHeuristicCost( Parameters.VolumeNavigationData, from, to ) * Parameters.NavigationQueryFilter.GetHeuristicScale();
}

float FSVOPathFindingAlgorithmStepper::GetTraversalCost( const FSVONodeAddress & from, const FSVONodeAddress & to ) const
{
    return Parameters.CostCalculator->GetTraversalCost( Parameters.VolumeNavigationData, from, to );
}

ENavigationQueryResult::Type USVOPathFindingAlgorithm::GetPath( FSVONavigationPath & /*navigation_path*/, const FSVOPathFindingParameters & /*params*/ ) const
{
    return ENavigationQueryResult::Error;
}

TSharedPtr< FSVOPathFindingAlgorithmStepper > USVOPathFindingAlgorithm::GetDebugPathStepper( FSVOPathFinderDebugInfos & /*debug_infos*/, const FSVOPathFindingParameters /*params*/ ) const
{
    return nullptr;
}
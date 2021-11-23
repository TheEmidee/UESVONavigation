#include "PathFinding/SVOPathFindingAlgorithm.h"

#include "PathFinding/SVOPathHeuristicCalculator.h"
#include "PathFinding/SVOPathTraversalCostCalculator.h"
#include "Pathfinding/SVONavigationQueryFilterImpl.h"
#include "SVONavigationData.h"
#include "SVOVolumeNavigationData.h"

FSVONodeAddressWithLocation::FSVONodeAddressWithLocation( const FSVONodeAddress & node_address, const FVector & location ) :
    NodeAddress( node_address ),
    Location( location )
{}

FSVONodeAddressWithLocation::FSVONodeAddressWithLocation( const FSVONodeAddress & node_address, const FSVOVolumeNavigationData & bounds_navigation_data ) :
    NodeAddress( node_address ),
    Location( bounds_navigation_data.GetNodePositionFromAddress( node_address ) )
{}

FSVOPathFinderDebugNodeCost::FSVOPathFinderDebugNodeCost( const FSVONodeAddressWithLocation & from, const FSVONodeAddressWithLocation & to, const float cost, const bool is_closed ) :
    From( from ),
    To( to ),
    Cost( cost ),
    bIsClosed( is_closed )
{
}

void FSVOPathFinderDebugNodeCost::Reset()
{
    *this = FSVOPathFinderDebugNodeCost();
}

void FSVOPathFinderDebugInfos::Reset()
{
    LastProcessedSingleNode.Reset();
    ProcessedNeighbors.Reset();
    Iterations = 0;
    VisitedNodes = 0;
    CurrentBestPath.ResetForRepath();
}

FSVOPathFindingParameters::FSVOPathFindingParameters( const FSVOVolumeNavigationData & volume_navigation_data, const FVector & start_location, const FVector & end_location, const FNavigationQueryFilter & nav_query_filter ) :
    StartLocation( start_location ),
    EndLocation( end_location ),
    NavigationQueryFilter( nav_query_filter ),
    QueryFilterImplementation( static_cast< const FSVONavigationQueryFilterImpl * >( NavigationQueryFilter.GetImplementation() ) ),
    QueryFilterSettings( QueryFilterImplementation->QueryFilterSettings ),
    HeuristicCalculator( QueryFilterSettings.HeuristicCalculator ),
    CostCalculator( QueryFilterSettings.TraversalCostCalculator ),
    VolumeNavigationData( volume_navigation_data )
{
    VolumeNavigationData.GetNodeAddressFromPosition( StartNodeAddress, StartLocation );
    VolumeNavigationData.GetNodeAddressFromPosition( EndNodeAddress, EndLocation );
}

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

bool FSVOPathFindingAlgorithmStepper::FillNodeAddresses( TArray< FSVONodeAddress > & node_addresses ) const
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
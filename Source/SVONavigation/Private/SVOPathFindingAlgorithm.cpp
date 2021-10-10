
#include "SVOPathFindingAlgorithm.h"

#include "SVONavigationData.h"
#include "SVONavigationQueryFilterImpl.h"
#include "SVONavigationTypes.h"
#include "SVOPathCostCalculator.h"
#include "SVOPathHeuristicCalculator.h"

namespace
{
    void ApplyVerticalOffset( FNavigationPath & path, const float vertical_offset )
    {
        auto & path_points = path.GetPathPoints();

        for (auto & path_point : path_points)
        {
            path_point.Location.Z += vertical_offset;
        }
    }

    void BuildPath( FNavigationPath & path, const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start_link, const FSVOOctreeLink & end_link, const TMap< FSVOOctreeLink, FSVOOctreeLink > & came_from )
    {
        auto & path_points = path.GetPathPoints();

        auto current = end_link;

        const auto get_link_position = []( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & link ) {
            FVector link_position;
            bounds_data.GetLinkPosition( link_position, link );
            return link_position;
        };

        while (current != start_link)
        {
            current = came_from[ current ];
            path_points.Emplace( get_link_position( bounds_data, current ) );
        }

        path_points.Emplace( get_link_position( bounds_data, start_link ) );

        Algo::Reverse( path_points );
    }

    void AdjustPathEnds( FNavigationPath & path, const FVector & start_location, const FVector & end_location )
    {
        auto & path_points = path.GetPathPoints();
        path_points[ 0 ].Location = start_location;

        if (path_points.Num() > 1)
        {
            path_points.Top().Location = end_location;
        }
    }
}

void FSVOPathFinderDebugInfos::Reset()
{
    DebugSteps.Reset();
    CurrentBestPath.ResetForRepath();
    Iterations = 0;
    VisitedNodes = 0;
}

FSVOPathFindingAlgorithmStepper::FSVOPathFindingAlgorithmStepper( FNavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query ) :
    NavigationPath( navigation_path ),
    NavigationData( navigation_data ),
    StartLocation( start_location ),
    EndLocation( end_location ),
    NavigationQueryFilter( *path_finding_query.QueryFilter ),
    QueryFilterImplementation( static_cast< const FSVONavigationQueryFilterImpl * >( path_finding_query.QueryFilter->GetImplementation() ) ),
    QueryFilterSettings( QueryFilterImplementation->QueryFilterSettings ),
    HeuristicCalculator( QueryFilterSettings.PathHeuristicCalculatorClass->GetDefaultObject<USVOPathHeuristicCalculator>() ),
    CostCalculator( QueryFilterSettings.PathCostCalculatorClass->GetDefaultObject<USVOPathCostCalculator>() ),
    BoundsNavigationData( navigation_data.GetSVOData().GetBoundsNavigationDataContainingPoints( { start_location, end_location } ) ),
    VerticalOffset( QueryFilterSettings.bOffsetPathVerticallyByAgentRadius ? -path_finding_query.NavAgentProperties.AgentRadius : 0.0f )
{
    if ( BoundsNavigationData != nullptr )
    {
        BoundsNavigationData->GetLinkFromPosition( StartLink, StartLocation );
        BoundsNavigationData->GetLinkFromPosition( EndLink, EndLocation );
    } 
}

ENavigationQueryResult::Type USVOPathFindingAlgorithm::GetPath( FNavigationPath & navigation_path, const ASVONavigationData & /*navigation_data*/, const FVector & /*start_location*/, const FVector & /*end_location*/, const FPathFindingQuery & path_finding_query ) const
{
    navigation_path.ResetForRepath();
    return ENavigationQueryResult::Fail;
}

TSharedPtr< FSVOPathFindingAlgorithmStepper > USVOPathFindingAlgorithm::GetDebugPathStepper( FNavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query )
{
    return MakeShared< FSVOPathFindingAlgorithmStepper >( navigation_path, navigation_data, start_location, end_location, path_finding_query );
}

FSVOPathFindingAlgorithmStepperAStar::FSVOPathFindingAlgorithmStepperAStar( FNavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query ) :
    FSVOPathFindingAlgorithmStepper( navigation_path, navigation_data, start_location, end_location, path_finding_query )
{
    if ( StartLink.IsValid() && EndLink.IsValid() )
    {
        Frontier.Emplace( StartLink, HeuristicCalculator->GetHeuristicCost( *BoundsNavigationData, StartLink, EndLink ) * NavigationQueryFilter.GetHeuristicScale() );
        CameFrom.Add( StartLink, StartLink );
        CostSoFar.Add( StartLink, 0.0f );        
    }
}

template <typename _VISITOR_TYPE_>
TSVOPathFindingAlgorithmStepperAStar<_VISITOR_TYPE_>::TSVOPathFindingAlgorithmStepperAStar( FNavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query ) :
    FSVOPathFindingAlgorithmStepperAStar( navigation_path, navigation_data, start_location, end_location, path_finding_query ),
    Visitor( *this )
{   
}

template <typename _VISITOR_TYPE_>
bool TSVOPathFindingAlgorithmStepperAStar<_VISITOR_TYPE_>::Step( ENavigationQueryResult::Type & result )
{
    if (BoundsNavigationData == nullptr
        || !StartLink.IsValid()
        || !EndLink.IsValid()
        || Frontier.Num() == 0
    )
    {
        result = ENavigationQueryResult::Fail;
        return false;
    }

    const auto current = Frontier.Pop();

    Visitor.OnOpenNode( current );

    if (current.Link == EndLink)
    {
        result = ENavigationQueryResult::Success;
        Visitor.OnEndLinkReached();
        return false;
    }

    TArray< FSVOOctreeLink > neighbors;
    BoundsNavigationData->GetNeighbors( neighbors, current.Link );

    for (const auto & neighbor : neighbors)
    {
        const auto neighbor_cost = CostCalculator->GetCost( *BoundsNavigationData, current.Link, neighbor );
        const auto new_cost = CostSoFar[ current.Link ] + neighbor_cost;

        // FSVOPathFinderDebugCost neighbor_debug_cost;
        // neighbor_debug_cost.Cost = neighbor_cost;
        // BoundsNavigationData->GetLinkPosition( neighbor_debug_cost.Location, neighbor );

        if (!CostSoFar.Contains( neighbor ) || new_cost < CostSoFar[ neighbor ])
        {
            CostSoFar.FindOrAdd( neighbor ) = new_cost;
            const auto priority = new_cost + HeuristicCalculator->GetHeuristicCost( *BoundsNavigationData, neighbor, EndLink ) * NavigationQueryFilter.GetHeuristicScale();
            Frontier.Emplace( neighbor, priority );
            Frontier.Sort();
            CameFrom.FindOrAdd( neighbor ) = current.Link;

            Visitor.OnNeighborEvaluated();

            // debug_infos.CurrentBestPath.ResetForRepath();
            // BuildPath( debug_infos.CurrentBestPath, *BoundsNavigationData, StartLink, current.Link, CameFrom );
            //
            // neighbor_debug_cost.WasEvaluated = true;
        }

        Visitor.OnNeighborExpanded();
        // debug_step.NeighborLocationCosts.Emplace( MoveTemp( neighbor_debug_cost ) );
        // debug_infos.VisitedNodes++;
    }

    Visitor.OnEndStep();

    // debug_infos.DebugSteps.Emplace( MoveTemp( debug_step ) );
    // debug_infos.Iterations++;

    return true;
}

struct FSVOPathFindingAlgorithmStepperAStarVisitorDefault
{
    
};

// void FSVOPathFindingAlgorithmStepperAStarVisitorDebug::OnOpenNode( const FSVOLinkWithCost & link_with_cost )
// {
//     FSVOPathFinderDebugStep debug_step;
//     debug_step.CurrentLocationCost.Cost = link_with_cost.Cost;
//     //Stepper.GetBoundsNavigationData().GetLinkPosition( debug_step.CurrentLocationCost.Location, link_with_cost.Link );
// }
//
// void FSVOPathFindingAlgorithmStepperAStarVisitorDebug::OnEndLinkReached()
// {
// }
//
void FSVOPathFindingAlgorithmStepperAStarVisitorPathConstructor::OnOpenNode( const FSVOLinkWithCost & link_with_cost )
{
}

void FSVOPathFindingAlgorithmStepperAStarVisitorPathConstructor::OnEndLinkReached()
{
    auto & navigation_path = Stepper.GetNavigationPath(); 

    BuildPath( navigation_path, *Stepper.GetBoundsNavigationData(), Stepper.GetStartLink(), Stepper.GetEndLink(), Stepper.GetCameFrom() );
    AdjustPathEnds( navigation_path, Stepper.GetStartLocation(), Stepper.GetEndLocation() );

    if ( Stepper.GetVerticalOffset() != 0.0f )
    {
        ApplyVerticalOffset( navigation_path, Stepper.GetVerticalOffset() );
    }
    
    navigation_path.MarkReady();
}

void FSVOPathFindingAlgorithmStepperAStarVisitorPathConstructor::OnNeighborEvaluated()
{
}

void FSVOPathFindingAlgorithmStepperAStarVisitorPathConstructor::OnNeighborExpanded()
{
}

void FSVOPathFindingAlgorithmStepperAStarVisitorPathConstructor::OnEndStep()
{
}

ENavigationQueryResult::Type USVOPathFindingAlgorithmAStar::GetPath( FNavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query ) const
{
    const auto stepper = MakeShared< TSVOPathFindingAlgorithmStepperAStar< FSVOPathFindingAlgorithmStepperAStarVisitorPathConstructor > >( navigation_path, navigation_data, start_location, end_location, path_finding_query );
    
    int iterations = 0;

    ENavigationQueryResult::Type result = ENavigationQueryResult::Fail;

    while ( stepper->Step( result ) )
    {
        iterations++;
    }

    return result;
}

TSharedPtr< FSVOPathFindingAlgorithmStepper > USVOPathFindingAlgorithmAStar::GetDebugPathStepper( FNavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query )
{
    // static FSVOPathFindingAlgorithmStepperAStarVisitorDebug StepperVisitorDebug;
    // return MakeShared< FSVOPathFindingAlgorithmStepperAStar< FSVOPathFindingAlgorithmStepperAStarVisitorDebug > >( StepperVisitorDebug, navigation_data, start_location, end_location, path_finding_query );

    return nullptr;
}
#include "SVOPathFinder.h"

#include "SVONavigationData.h"
#include "SVONavigationTypes.h"

FSVOPathFinder::FSVOPathFinder( const ASVONavigationData & navigation_data ) :
    NavigationData( navigation_data )
{
}

ENavigationQueryResult::Type FSVOPathFinder::GetPath( FNavigationPath & navigation_path, const FVector & start_location, const FVector & end_location, const FNavigationQueryFilter & navigation_query_filter )
{
    const auto & navigation_bounds_data = NavigationData.GetNavigationBoundsData();

    const auto * bounds_data = navigation_bounds_data.FindByPredicate( [ this, &start_location, &end_location ]( const FSVOBoundsNavigationData & data ) {
        return data.GetNavigationBounds().IsInside( start_location ) && data.GetNavigationBounds().IsInside( end_location );
    } );

    if ( bounds_data == nullptr )
    {
        return ENavigationQueryResult::Fail;
    }

    FSVOOctreeLink start_link;
    bounds_data->GetLinkFromPosition( start_link, start_location );
    if ( !start_link.IsValid() )
    {
        return ENavigationQueryResult::Fail;
    }

    FSVOOctreeLink end_link;
    bounds_data->GetLinkFromPosition( end_link, end_location );
    if ( !end_link.IsValid() )
    {
        return ENavigationQueryResult::Fail;
    }

    struct FSVOLinkWithCost
    {
        FSVOLinkWithCost( const FSVOOctreeLink & link, float cost ) :
            Link( link ),
            Cost( cost )
        {
        }

        FSVOOctreeLink Link;
        float Cost;

        bool operator<( const FSVOLinkWithCost & other ) const
        {
            return Cost < other.Cost;
        }
    };

    TArray< FSVOLinkWithCost > frontier;
    TArray< FSVOOctreeLink > neighbors;
    TMap< FSVOOctreeLink, FSVOOctreeLink > came_from;
    TMap< FSVOOctreeLink, float > cost_so_far;

    frontier.Emplace( start_link, 0.0f );
    came_from.Add( start_link, start_link );
    cost_so_far.Add( start_link, 0.0f );

    while ( frontier.Num() > 0 )
    {
        const auto current = frontier.Pop();

        if ( current.Link == end_link )
        {
            auto result = BuildPath( navigation_path, *bounds_data, start_link, end_link, came_from );

            if ( result == ENavigationQueryResult::Success )
            {
                auto & path_points = navigation_path.GetPathPoints();
                path_points[ 0 ].Location = start_location;

                if ( path_points.Num() > 1 )
                {
                    path_points.Top().Location = end_location;
                }

                navigation_path.MarkReady();
            }

            return result;
        }

        bounds_data->GetNeighbors( neighbors, current.Link );

        for ( const auto & next : neighbors )
        {
            const auto new_cost = cost_so_far[ current.Link ] + GetCost( *bounds_data, current.Link, next );
            if ( !cost_so_far.Contains( next ) || new_cost < cost_so_far[ next ] )
            {
                cost_so_far.FindOrAdd( next ) = new_cost;
                const auto priority = new_cost + GetHeuristicScore( *bounds_data, next, end_link );
                frontier.Emplace( next, priority );
                frontier.Sort( []( const auto & left, const auto & right )
                {
                    return left.Cost > right.Cost;
                });
                came_from.FindOrAdd( next ) = current.Link;
            }
        }
    }

    /*typedef typename Graph::Location Location;
    typedef typename Graph::cost_t cost_t;
    PriorityQueue< Location, cost_t > frontier;
    std::vector< Location > neighbors;
    frontier.put( start, cost_t( 0 ) );

    came_from[ start ] = start;
    cost_so_far[ start ] = cost_t( 0 );

    while ( !frontier.empty() )
    {
        typename Location current = frontier.get();

        if ( current == goal )
        {
            break;
        }

        graph.get_neighbors( current, neighbors );
        for ( Location next : neighbors )
        {
            cost_t new_cost = cost_so_far[ current ] + graph.cost( current, next );
            if ( cost_so_far.find( next ) == cost_so_far.end() || new_cost < cost_so_far[ next ] )
            {
                cost_so_far[ next ] = new_cost;
                cost_t priority = new_cost + heuristic( next, goal );
                frontier.put( next, priority );
                came_from[ next ] = current;
            }
        }
    }*/

    /*
    OpenSet.Add( start_link );
    Parent.Add( start_link, start_link );
    GScore.Add( start_link, 0 );
    FScore.Add( start_link, GetHeuristicScore( start_link, EndLink ) ); // Distance to target

    int numIterations = 0;

    FSVOOctreeLink current_link;

    while ( OpenSet.Num() > 0 )
    {
        float lowest_score = FLT_MAX;
        for ( auto & link : OpenSet )
        {
            if ( !FScore.Contains( link ) || FScore[ link ] < lowest_score )
            {
                lowest_score = FScore[ link ];
                current_link = link;
            }
        }

        OpenSet.Remove( current_link );
        ClosedSet.Add( current_link );

        if ( current_link == EndLink )
        {
            BuildPath( navigation_path, current_link, *BoundsData );
            return ENavigationQueryResult::Success;
        }

        const auto & current_node = BoundsData->GetNodeFromLink( current_link );

        TArray< FSVOOctreeLink > neighbor_links;

        if ( current_link.LayerIndex == 0 && current_node.FirstChild.IsValid() )
        {
            BoundsData->GetLeafNeighbors( neighbor_links, current_link );
        }
        else
        {
            BoundsData->GetNeighbors( neighbor_links, current_link );
        }

        for ( const auto & neighbor : neighbor_links )
        {
            ProcessLink( current_link, neighbor );
        }

        numIterations++;
    }
    */

    return ENavigationQueryResult::Fail;
}

ENavigationQueryResult::Type FSVOPathFinder::BuildPath( FNavigationPath & path, const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start_link, const FSVOOctreeLink & end_link, const TMap< FSVOOctreeLink, FSVOOctreeLink > & came_from ) const
{
    auto & path_points = path.GetPathPoints();

    auto current = end_link;

    const auto get_link_position = []( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & link ) {
        FVector link_position;
        bounds_data.GetLinkPosition( link_position, link );
        return link_position;
    };

    while ( current != start_link )
    {
        if ( auto * new_current = came_from.Find( current ) )
        {
            path_points.Emplace( get_link_position( bounds_data, current ) );
            current = *new_current;
        }
        else
        {
            return ENavigationQueryResult::Error;
        }
    }

    path_points.Emplace( get_link_position( bounds_data, start_link ) );

    Algo::Reverse( path_points );

    return ENavigationQueryResult::Success;

    /*while ( Parent.Contains( current ) && !( current == Parent[ current ] ) )
    {
        current = Parent[ current ];

        FVector link_position;
        bounds_navigation_data.GetLinkPosition( link_position, current );

        path_points.Add( FNavPathPoint( link_position ) );*/

    //const auto & node = bounds_navigation_data.GetNodeFromLink( current );
    //// This is rank. I really should sort the layers out
    //if ( current.GetLayerIndex() == 0 )
    //{
    //    if ( !node.HasChildren() )
    //        points[ points.Num() - 1 ].myLayer = 1;
    //    else
    //        points[ points.Num() - 1 ].myLayer = 0;
    //}
    //else
    //{
    //    points[ points.Num() - 1 ].myLayer = current.GetLayerIndex() + 1;
    //}
    //}

    //if ( path_points.Num() > 1 )
    //{
    //    path_points[ 0 ].Location = EndLocation;
    //    path_points[ path_points.Num() - 1 ].Location = StartLocation;
    //}
    //else // If start and end are in the same voxel, just use the start and target positions.
    //{
    //    if ( path_points.Num() == 0 )
    //    {
    //        path_points.Emplace();
    //    }

    //    path_points[ 0 ].Location = EndLocation;
    //    path_points.Emplace( StartLocation );
    //}
}

//void FSVOPathFinder::ProcessLink( const FSVOOctreeLink & current_link, const FSVOOctreeLink & neighbor_link )
//{
//    if ( !neighbor_link.IsValid() )
//    {
//        return;
//    }
//
//    if ( ClosedSet.Contains( neighbor_link ) )
//    {
//        return;
//    }
//
//    if ( !OpenSet.Contains( neighbor_link ) )
//    {
//        OpenSet.Add( neighbor_link );
//
//        /*if ( mySettings.myDebugOpenNodes )
//        {
//            FVector pos;
//            myVolume.GetLinkPosition( neighbor_link, pos );
//            mySettings.myDebugPoints.Add( pos );
//        }*/
//    }
//
//    float g_score = FLT_MAX;
//    if ( GScore.Contains( current_link ) )
//    {
//        g_score = GScore[ current_link ] + GetCost( current_link, neighbor_link );
//    }
//    else
//    {
//        GScore.Add( current_link, FLT_MAX );
//    }
//
//    if ( g_score >= ( GScore.Contains( neighbor_link ) ? GScore[ neighbor_link ] : FLT_MAX ) )
//    {
//        return;
//    }
//
//    Parent.Add( neighbor_link, current_link );
//    GScore.Add( neighbor_link, g_score );
//    FScore.Add( neighbor_link, GScore[ neighbor_link ] + ( /*mySettings.myEstimateWeight **/ GetHeuristicScore( neighbor_link, EndLink ) ) );
//}

float FSVOPathFinder::GetHeuristicScore( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const
{
    float score = 0.0f;

    FVector start_location;
    FVector end_location;

    bounds_data.GetLinkPosition( start_location, start );
    bounds_data.GetLinkPosition( end_location, end );

    /*switch (mySettings.myPathCostType)
	{
	case ESVONPathCostType::MANHATTAN:*/
    score = FMath::Abs( end_location.X - start_location.X ) + FMath::Abs( end_location.Y - start_location.Y ) + FMath::Abs( end_location.Z - start_location.Z );
    /*	break;
	case ESVONPathCostType::EUCLIDEAN:
	default:
		score = (start_location - end_location).Size();
		break;
	}*/

    //score *= ( 1.0f - ( static_cast< float >( aTarget.GetLayerIndex() ) / static_cast< float >( myVolume.GetMyNumLayers() ) ) * mySettings.myNodeSizeCompensation );

    return score;
}

float FSVOPathFinder::GetCost( const FSVOBoundsNavigationData & bounds_data, const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const
{
    float cost = 0.f;

    // Unit cost implementation
    /* if ( mySettings.myUseUnitCost )
    {
        cost = mySettings.myUnitCost;
    }
    else*/
    {

        FVector start_location( 0.0f );
        FVector end_location( 0.0f );

        bounds_data.GetLinkPosition( start_location, start );
        bounds_data.GetLinkPosition( end_location, end );
        cost = ( start_location - end_location ).Size();
    }

    //cost *= ( 1.0f - ( static_cast< float >( aTarget.GetLayerIndex() ) / static_cast< float >( myVolume.GetMyNumLayers() ) ) * mySettings.myNodeSizeCompensation );

    return cost;
}

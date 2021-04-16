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
                frontier.Sort( []( const auto & left, const auto & right ) {
                    return left.Cost > right.Cost;
                } );
                came_from.FindOrAdd( next ) = current.Link;
            }
        }
    }

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
}

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

#include "SVOPathFinder.h"

#include "SVONavigationData.h"
#include "SVONavigationTypes.h"

struct FNav3DPathPoint
{
    FVector Location;
    int32 Layer;

    FNav3DPathPoint() :
        Location( FVector::ZeroVector ),
        Layer( -1 )
    {}
    FNav3DPathPoint( const FVector & Location, const int32 LayerIndex ) :
        Location( Location ),
        Layer( LayerIndex )
    {}
};

struct FNav3DPath
{
    TArray< FNav3DPathPoint > Points;
};

FSVOPathFinder::FSVOPathFinder( const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FNavigationQueryFilter & navigation_query_filter ) :
    NavigationData( navigation_data ),
    StartLocation( start_location ),
    EndLocation( end_location ),
    NavigationQueryFilter( navigation_query_filter )
{
}

FPathFindingResult FSVOPathFinder::GetPath( FNavigationPath & navigation_path )
{
    const auto & navigation_bounds_data = NavigationData.GetNavigationBoundsData();

    BoundsData = navigation_bounds_data.FindByPredicate( [ this ]( const FSVOBoundsNavigationData & data ) {
        return data.GetNavigationBounds().IsInside( StartLocation ) && data.GetNavigationBounds().IsInside( EndLocation );
    } );

    if ( BoundsData == nullptr )
    {
        return ENavigationQueryResult::Fail;
    }

    const auto start_link = BoundsData->GetLinkFromPosition( StartLocation ).Get( FSVOOctreeLink() );
    if ( !start_link.IsValid() )
    {
        return ENavigationQueryResult::Fail;
    }

    EndLink = BoundsData->GetLinkFromPosition( EndLocation ).Get( FSVOOctreeLink() );
    if ( !EndLink.IsValid() )
    {
        return ENavigationQueryResult::Fail;
    }

    FPathFindingResult result;

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
            BuildPath( *result.Path, current_link, *BoundsData );
            result.Result = ENavigationQueryResult::Success;
            return result;
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

    return result;
}

void FSVOPathFinder::BuildPath( FNavigationPath & path, const FSVOOctreeLink & link, const FSVOBoundsNavigationData & bounds_navigation_data ) const
{
    auto & path_points = path.GetPathPoints();

    auto current = link;
    while ( Parent.Contains( current ) && !( current == Parent[ current ] ) )
    {
        current = Parent[ current ];

        FVector link_position;
        bounds_navigation_data.GetLinkPosition( link_position, current );

        path_points.Add( FNavPathPoint( link_position ) );

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
    }

    if ( path_points.Num() > 1 )
    {
        path_points[ 0 ].Location = EndLocation;
        path_points[ path_points.Num() - 1 ].Location = StartLocation;
    }
    else // If start and end are in the same voxel, just use the start and target positions.
    {
        if ( path_points.Num() == 0 )
        {
            path_points.Emplace();
        }

        path_points[ 0 ].Location = EndLocation;
        path_points.Emplace( StartLocation );
    }
}

void FSVOPathFinder::ProcessLink( const FSVOOctreeLink & current_link, const FSVOOctreeLink & neighbor_link )
{
    if ( !neighbor_link.IsValid() )
    {
        return;
    }

    if ( ClosedSet.Contains( neighbor_link ) )
    {
        return;
    }

    if ( !OpenSet.Contains( neighbor_link ) )
    {
        OpenSet.Add( neighbor_link );

        /*if ( mySettings.myDebugOpenNodes )
        {
            FVector pos;
            myVolume.GetLinkPosition( neighbor_link, pos );
            mySettings.myDebugPoints.Add( pos );
        }*/
    }

    float g_score = FLT_MAX;
    if ( GScore.Contains( current_link ) )
    {
        g_score = GScore[ current_link ] + GetCost( current_link, neighbor_link );
    }
    else
    {
        GScore.Add( current_link, FLT_MAX );
    }

    if ( g_score >= ( GScore.Contains( neighbor_link ) ? GScore[ neighbor_link ] : FLT_MAX ) )
    {
        return;
    }

    Parent.Add( neighbor_link, current_link );
    GScore.Add( neighbor_link, g_score );
    FScore.Add( neighbor_link, GScore[ neighbor_link ] + ( /*mySettings.myEstimateWeight **/ GetHeuristicScore( neighbor_link, EndLink ) ) );
}

float FSVOPathFinder::GetHeuristicScore( const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const
{
    float score = 0.0f;

    FVector start_location;
    FVector end_location;

    BoundsData->GetLinkPosition( start_location, start );
    BoundsData->GetLinkPosition( end_location, end );

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

float FSVOPathFinder::GetCost( const FSVOOctreeLink & start, const FSVOOctreeLink & end ) const
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

        const auto & start_node = BoundsData->GetNodeFromLink( start );
        const auto & end_node = BoundsData->GetNodeFromLink( end );

        BoundsData->GetLinkPosition( start_location, start );
        BoundsData->GetLinkPosition( end_location, end );
        cost = ( start_location - end_location ).Size();
    }

    //cost *= ( 1.0f - ( static_cast< float >( aTarget.GetLayerIndex() ) / static_cast< float >( myVolume.GetMyNumLayers() ) ) * mySettings.myNodeSizeCompensation );

    return cost;
}

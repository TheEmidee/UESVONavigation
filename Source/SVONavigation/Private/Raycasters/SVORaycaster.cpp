#include "Raycasters/SVORayCaster.h"

#include "SVONavigationData.h"
#include "SVOVolumeNavigationData.h"

#include <NavigationSystem.h>

FSVORayCasterObserver_GenerateDebugInfos::FSVORayCasterObserver_GenerateDebugInfos( FSVORayCasterDebugInfos & debug_infos ) :
    DebugInfos( debug_infos )
{
}

void FSVORayCasterObserver_GenerateDebugInfos::Initialize( const FSVOVolumeNavigationData * navigation_data, const FVector from, const FVector to )
{
    DebugInfos.TraversedNodes.Reset();
    DebugInfos.TraversedLeafNodes.Reset();
    DebugInfos.TraversedLeafSubNodes.Reset();
    DebugInfos.RayCastStartLocation = from;
    DebugInfos.RayCastEndLocation = to;
    DebugInfos.NavigationData = navigation_data;
}

void FSVORayCasterObserver_GenerateDebugInfos::SetResult( const bool result )
{
    DebugInfos.Result = result;
}

void FSVORayCasterObserver_GenerateDebugInfos::AddTraversedNode( FSVONodeAddress node_address, bool is_occluded )
{
    UE_LOG( LogTemp, Warning, TEXT( "Node Address : %i - %i - %i" ), node_address.LayerIndex, node_address.NodeIndex, node_address.SubNodeIndex );
    DebugInfos.TraversedNodes.Emplace( node_address, is_occluded );
}

void FSVORayCasterObserver_GenerateDebugInfos::AddTraversedLeafNode( FSVONodeAddress node_address, bool is_occluded )
{
#if WITH_EDITOR
    const auto & layer_zero = DebugInfos.NavigationData->GetData().GetLayer( 0 );
    const auto & layer_zero_nodes = layer_zero.GetNodes();

    if ( const auto * node_ptr = layer_zero_nodes.FindByPredicate( [ &node_address ]( const FSVONode & layer_zero_node ) {
             return layer_zero_node.FirstChild == node_address;
         } ) )
    {
        const FSVONodeAddress leaf_node_address( 0, node_ptr->MortonCode );
        UE_LOG( LogTemp, Warning, TEXT( "LeafNode Address : 0 %i - Index : %i" ), leaf_node_address.NodeIndex, node_address.NodeIndex );
    }
    else
    {
        UE_LOG( LogTemp, Warning, TEXT( "LeafNode Address not found for index : %i" ), node_address.NodeIndex );
    }
#endif

    DebugInfos.TraversedLeafNodes.Emplace( node_address, is_occluded );
}

void FSVORayCasterObserver_GenerateDebugInfos::AddTraversedLeafSubNode( FSVONodeAddress node_address, bool is_occluded )
{
    UE_LOG( LogTemp, Warning, TEXT( "SubNode Address : %i - %i - %i" ), node_address.LayerIndex, node_address.NodeIndex, node_address.SubNodeIndex );
    DebugInfos.TraversedLeafSubNodes.Emplace( node_address, is_occluded );
}

bool USVORayCaster::Trace( UObject * world_context, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const
{
    if ( UNavigationSystemV1 * navigation_system = UNavigationSystemV1::GetCurrent( world_context->GetWorld() ) )
    {
        if ( auto * navigation_data = navigation_system->GetNavDataForProps( nav_agent_properties ) )
        {
            if ( const auto * svo_navigation_data = Cast< ASVONavigationData >( navigation_data ) )
            {
                if ( const auto * volume_navigation_data = svo_navigation_data->GetVolumeNavigationDataContainingPoints( { from, to } ) )
                {
                    return Trace( world_context, *volume_navigation_data, from, to, nav_agent_properties );
                }
            }
        }
    }

    return false;
}

bool USVORayCaster::Trace( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FSVONodeAddress from, const FSVONodeAddress to, const FNavAgentProperties & nav_agent_properties ) const
{
    const auto from_position = volume_navigation_data.GetNodePositionFromAddress( from );
    const auto to_position = volume_navigation_data.GetNodePositionFromAddress( to );

    return Trace( world_context, volume_navigation_data, from_position, to_position, nav_agent_properties );
}

bool USVORayCaster::Trace( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const
{
    if ( Observer.IsValid() )
    {
        Observer->Initialize( &volume_navigation_data, from, to );
    }

    const auto result = TraceInternal( world_context, volume_navigation_data, from, to, nav_agent_properties );

    if ( Observer.IsValid() )
    {
        Observer->SetResult( result );
    }

    return result;
}

void USVORayCaster::SetObserver( const TSharedPtr< FSVORayCasterObserver > observer )
{
    Observer = observer;
}

bool USVORayCaster::TraceInternal( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const
{
    return false;
}

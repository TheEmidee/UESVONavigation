#include "Raycasters/SVORayCaster.h"

#include "SVONavigationData.h"
#include "SVOVolumeNavigationData.h"


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

void FSVORayCasterObserver_GenerateDebugInfos::AddTraversedLeafSubNode( FSVONodeAddress node_address, bool is_occluded )
{
    UE_LOG( LogTemp, Warning, TEXT( "SubNode Address : %i - %i - %i" ), node_address.LayerIndex, node_address.NodeIndex, node_address.SubNodeIndex );
    DebugInfos.TraversedLeafSubNodes.Emplace( node_address, is_occluded );
}

bool USVORayCaster::Trace( const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to ) const
{
    if ( Observer.IsValid() )
    {
        Observer->Initialize( &volume_navigation_data, from, to );
    }

    const auto result = TraceInternal( volume_navigation_data, from, to );

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

bool USVORayCaster::TraceInternal( const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to ) const
{
    return false;
}

UWorld * USVORayCaster::GetWorldContext()
{
#if WITH_EDITOR
    return GEditor->GetEditorWorldContext( false ).World();
#else
    return GEngine->GetCurrentPlayWorld();
#endif
}

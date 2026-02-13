#include "SVOBlueprintFunctionLibrary.h"

#include "SVONavigationData.h"

#include <AI/NavigationSystemBase.h>
#include <NavigationSystem.h>

bool USVOBlueprintFunctionLibrary::IsActorInNavigableArea( const AActor * actor )
{
    if ( actor == nullptr )
    {
        return false;
    }

    const auto * world = actor->GetWorld();
    if ( world == nullptr )
    {
        return false;
    }

    auto * nav_system = Cast< UNavigationSystemV1 >( world->GetNavigationSystem() );
    if ( nav_system == nullptr )
    {
        return false;
    }

    const auto * nav_data = Cast< ASVONavigationData >( nav_system->GetNavDataForActor( *actor ) );
    if ( nav_data == nullptr )
    {
        return false;
    }

    const TArray< FVector > points { actor->GetActorLocation() };
    auto * volume_nav_data = nav_data->GetVolumeNavigationDataContainingPoints( points );
    if ( volume_nav_data == nullptr )
    {
        return false;
    }

    FSVONodeAddress address;
    if ( !volume_nav_data->GetNodeAddressFromPosition( address, actor->GetActorLocation() ) )
    {
        return false;
    }

    return volume_nav_data->IsNodeAddressNavigable( address );
}
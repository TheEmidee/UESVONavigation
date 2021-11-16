#include "Raycasters/SVORaycaster.h"

#include "SVONavigationData.h"
#include "SVOVolumeNavigationData.h"

#include <NavigationSystem.h>

bool USVORaycaster::HasLineOfSight( UObject * world_context, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const
{
    if ( UNavigationSystemV1 * navigation_system = UNavigationSystemV1::GetCurrent( world_context->GetWorld() ) )
    {
        if ( auto * navigation_data = navigation_system->GetNavDataForProps( nav_agent_properties ) )
        {
            if ( const auto * svo_navigation_data = Cast< ASVONavigationData >( navigation_data ) )
            {
                if ( const auto * volume_navigation_data = svo_navigation_data->GetVolumeNavigationDataContainingPoints( { from, to } ) )
                {
                    return HasLineOfSight( world_context, *volume_navigation_data, from, to, nav_agent_properties );
                }
            }
        }
    }

    return false;
}

bool USVORaycaster::HasLineOfSight( UObject * /*world_context*/, const FSVOVolumeNavigationData & /*volume_navigation_data*/, const FSVONodeAddress /*from*/, const FSVONodeAddress /*to*/, const FNavAgentProperties & /*nav_agent_properties*/ ) const
{
    return false;
}

bool USVORaycaster::HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const
{
    return false;
}
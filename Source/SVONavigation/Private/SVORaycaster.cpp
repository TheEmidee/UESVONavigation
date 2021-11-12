#include "SVORaycaster.h"

#include "SVOVolumeNavigationData.h"

#include <Kismet/KismetSystemLibrary.h>
#include <DrawDebugHelpers.h>

bool USVORaycaster::HasLineOfSight( UObject * /*world_context*/, const FSVOVolumeNavigationData & /*volume_navigation_data*/, const FSVONodeAddress /*from*/, const FSVONodeAddress /*to*/, const FNavAgentProperties & /*nav_agent_properties*/ )
{
    return false;
}

bool USVORaycaster::HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties )
{
    return false;
}

bool USVORayCaster_PhysicsBase::HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FSVONodeAddress from, const FSVONodeAddress to, const FNavAgentProperties & nav_agent_properties )
{
    const auto from_position = volume_navigation_data.GetNodePositionFromAddress( from );
    const auto to_position = volume_navigation_data.GetNodePositionFromAddress( to );

    return HasLineOfSightInternal( world_context, from_position, to_position, nav_agent_properties );
}

bool USVORayCaster_PhysicsBase::HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties )
{
    return HasLineOfSightInternal( world_context, from, to, nav_agent_properties );
}

bool USVORayCaster_PhysicsBase::HasLineOfSightInternal( UObject * world_context, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const
{
    return false;
}

bool USVORayCaster_Ray::HasLineOfSightInternal( UObject * world_context, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const
{
    FHitResult hit_result;

    return !UKismetSystemLibrary::LineTraceSingle(
        world_context,
        from,
        to,
        TraceType,
        false,
        TArray< AActor * >(),
        bShowLineOfSightTraces ? EDrawDebugTrace::ForDuration : EDrawDebugTrace::None,
        hit_result,
        false,
        FLinearColor::Red,
        FLinearColor::Green,
        0.1f );
}

bool USVORayCaster_Sphere::HasLineOfSightInternal( UObject * world_context, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const
{
    FHitResult hit_result;

    return !UKismetSystemLibrary::SphereTraceSingle(
        world_context,
        from,
        to,
        nav_agent_properties.AgentRadius * AgentRadiusMultiplier,
        TraceType,
        false,
        TArray< AActor * >(),
        bShowLineOfSightTraces ? EDrawDebugTrace::ForDuration : EDrawDebugTrace::None,
        hit_result,
        false,
        FLinearColor::Red,
        FLinearColor::Green,
        0.1f );
}

bool USVORayCaster_OctreeTraversal::HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FSVONodeAddress from, const FSVONodeAddress to, const FNavAgentProperties & nav_agent_properties )
{
    return false;
}

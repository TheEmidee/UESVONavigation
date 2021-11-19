#include "Raycasters/SVORaycaster_Physics.h"

#include "SVOVolumeNavigationData.h"

#include <Kismet/KismetSystemLibrary.h>

bool USVORayCaster_PhysicsBase::TraceInternal( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const
{
    return TracePhysicsInternal( world_context, from, to, nav_agent_properties );
}

bool USVORayCaster_PhysicsBase::TracePhysicsInternal( UObject * world_context, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const
{
    return false;
}

bool USVORayCaster_Ray::TracePhysicsInternal( UObject * world_context, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const
{
    FHitResult hit_result;

    return UKismetSystemLibrary::LineTraceSingle(
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

bool USVORayCaster_Sphere::TracePhysicsInternal( UObject * world_context, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const
{
    FHitResult hit_result;

    return UKismetSystemLibrary::SphereTraceSingle(
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
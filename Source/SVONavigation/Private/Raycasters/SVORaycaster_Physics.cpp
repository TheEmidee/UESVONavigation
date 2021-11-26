#include "Raycasters/SVORaycaster_Physics.h"

#include "SVOVolumeNavigationData.h"

#include <Kismet/KismetSystemLibrary.h>

bool USVORayCaster_PhysicsBase::TraceInternal( const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to ) const
{
    return TracePhysicsInternal( from, to );
}

bool USVORayCaster_PhysicsBase::TracePhysicsInternal( const FVector & from, const FVector & to ) const
{
    return false;
}

bool USVORayCaster_Ray::TracePhysicsInternal( const FVector & from, const FVector & to ) const
{
    FHitResult hit_result;

    return UKismetSystemLibrary::LineTraceSingle(
        GetWorldContext(),
        from,
        to,
        TraceType,
        false,
        TArray< AActor * >(),
        bShowLineOfSightTraces ? EDrawDebugTrace::ForDuration : EDrawDebugTrace::None,
        hit_result,
        false,
        FLinearColor::Green,
        FLinearColor::Red,
        5.0f );
}

USVORayCaster_Sphere::USVORayCaster_Sphere()
{
    Radius = 50.0f;
}

bool USVORayCaster_Sphere::TracePhysicsInternal( const FVector & from, const FVector & to ) const
{
    FHitResult hit_result;

    return UKismetSystemLibrary::SphereTraceSingle(
        GetWorldContext(),
        from,
        to,
        Radius,
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

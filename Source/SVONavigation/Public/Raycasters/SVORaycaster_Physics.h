#pragma once

#include "SVORayCaster.h"

#include <CoreMinimal.h>

#include "SVORaycaster_Physics.generated.h"

UCLASS( Abstract )
class SVONAVIGATION_API USVORayCaster_PhysicsBase : public USVORayCaster
{
    GENERATED_BODY()

protected:

    bool TraceInternal( const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to ) const override;

    virtual bool TracePhysicsInternal( const FVector & from, const FVector & to ) const;

    UPROPERTY( EditAnywhere )
    TEnumAsByte< ETraceTypeQuery > TraceType;
};

UCLASS()
class SVONAVIGATION_API USVORayCaster_Ray final : public USVORayCaster_PhysicsBase
{
    GENERATED_BODY()

protected:

    bool TracePhysicsInternal( const FVector & from, const FVector & to ) const override;
};

UCLASS()
class SVONAVIGATION_API USVORayCaster_Sphere final : public USVORayCaster_PhysicsBase
{
    GENERATED_BODY()

public:

    USVORayCaster_Sphere();

protected:
    bool TracePhysicsInternal( const FVector & from, const FVector & to ) const override;

    UPROPERTY( EditAnywhere )
    float Radius;
};
#pragma once

#include "SVONavigationTypes.h"

#include <CoreMinimal.h>

#include "SVORaycaster.generated.h"

UCLASS( Abstract, NotBlueprintable, EditInlineNew )
class SVONAVIGATION_API USVORaycaster : public UObject
{
    GENERATED_BODY()

public:

    virtual bool HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FSVONodeAddress from, const FSVONodeAddress to, const FNavAgentProperties & nav_agent_properties );
};

UCLASS( Abstract )
class SVONAVIGATION_API USVORayCaster_PhysicsBase : public USVORaycaster
{
    GENERATED_BODY()

public:

    bool HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FSVONodeAddress from, const FSVONodeAddress to, const FNavAgentProperties & nav_agent_properties ) override;

protected:

    virtual bool HasLineOfSightInternal( UObject * world_context, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties) const;

    UPROPERTY( EditAnywhere )
    float AgentRadiusMultiplier;

    UPROPERTY( EditAnywhere )
    uint8 bShowLineOfSightTraces : 1;

    UPROPERTY( EditAnywhere )
    TEnumAsByte< ETraceTypeQuery > TraceType;
};

UCLASS()
class SVONAVIGATION_API USVORayCaster_Ray final : public USVORayCaster_PhysicsBase
{
    GENERATED_BODY()

protected:

    bool HasLineOfSightInternal( UObject * world_context, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const override;
};

UCLASS()
class SVONAVIGATION_API USVORayCaster_Sphere final : public USVORayCaster_PhysicsBase
{
    GENERATED_BODY()

protected:
    bool HasLineOfSightInternal( UObject * world_context, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const override;
};

UCLASS()
class SVONAVIGATION_API USVORayCaster_OctreeTraversal final : public USVORaycaster
{
    GENERATED_BODY()

public:

    bool HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FSVONodeAddress from, const FSVONodeAddress to, const FNavAgentProperties & nav_agent_properties ) override;
};
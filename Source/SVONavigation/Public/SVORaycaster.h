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
    virtual bool HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties );
};

UCLASS( Abstract )
class SVONAVIGATION_API USVORayCaster_PhysicsBase : public USVORaycaster
{
    GENERATED_BODY()

public:

    bool HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FSVONodeAddress from, const FSVONodeAddress to, const FNavAgentProperties & nav_agent_properties ) override;
    bool HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) override;

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

// This is an implementation of An Efficient Parametric Algorithm for Octree Traversal : http://wscg.zcu.cz/wscg2000/Papers_2000/X31.pdf
// Some code examples :
// https://github.com/kwstanths/Ray-traversal/blob/master/TrianglesOctree.hpp
// https://newbedev.com/ray-octree-intersection-algorithms
UCLASS()
class SVONAVIGATION_API USVORayCaster_OctreeTraversal final : public USVORaycaster
{
    GENERATED_BODY()

public:
    USVORayCaster_OctreeTraversal();

    bool HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FSVONodeAddress from, const FSVONodeAddress to, const FNavAgentProperties & nav_agent_properties ) override;
    bool HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) override;

private:

    struct FOctreeRay
    {
        FOctreeRay( float tx0, float tx1, float ty0, float ty1, float tz0, float tz1 );
        bool Intersects() const;

        float tx0;
        float tx1;
        float txm;
        float ty0;
        float ty1;
        float tym;
        float tz0;
        float tz1;
        float tzm;
    };

    static uint8 GetFirstNode( const FOctreeRay & ray );
    static uint8 GetNewNode( float txm, int32 x, float tym, int32 y, float tzm, int32 z );

    bool DoesRayIntersectLeaf( const FOctreeRay & ray, const FSVONodeAddress & node_address, const FSVOData & data ) const;
    bool DoesRayIntersectNormalNode( const FOctreeRay & ray, const FSVONodeAddress & node_address, const FSVOData & data ) const;
    bool DoesRayIntersectNode( const FOctreeRay & ray, const FSVONodeAddress & node_address, const FSVOData & data ) const;

    uint8 a = 0;

    UPROPERTY( EditAnywhere )
    uint8 bDrawDebug : 1;
};
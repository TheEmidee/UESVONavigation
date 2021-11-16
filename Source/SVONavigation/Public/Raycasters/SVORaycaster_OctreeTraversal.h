#pragma once

#include "SVORaycaster.h"

#include <CoreMinimal.h>

#include "SVORaycaster_OctreeTraversal.generated.h"

UCLASS()
class SVONAVIGATION_API USVORayCaster_OctreeTraversal final : public USVORaycaster
{
    GENERATED_BODY()

public:
    USVORayCaster_OctreeTraversal();

    bool HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FSVONodeAddress from, const FSVONodeAddress to, const FNavAgentProperties & nav_agent_properties ) const override;
    bool HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const override;

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

    static uint8 GetFirstNodeIndex( const FOctreeRay & ray );
    static uint8 GetNextNodeIndex( float txm, int32 x, float tym, int32 y, float tzm, int32 z );

    bool DoesRayIntersectLeaf( const FOctreeRay & ray, const FSVONodeAddress & node_address, const FSVOVolumeNavigationData & data ) const;
    bool DoesRayIntersectNormalNode( const FOctreeRay & ray, const FSVONodeAddress & node_address, const FSVOVolumeNavigationData & data ) const;
    bool DoesRayIntersectNode( const FOctreeRay & ray, const FSVONodeAddress & node_address, const FSVOVolumeNavigationData & data ) const;

    mutable uint8 a = 0;
    mutable UWorld * World;

    UPROPERTY( EditAnywhere )
    uint8 bDrawDebug : 1;
};
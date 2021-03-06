#pragma once

#include "SVORayCaster.h"

#include <CoreMinimal.h>

#include "SVORaycaster_OctreeTraversal.generated.h"

UCLASS()
class SVONAVIGATION_API USVORayCaster_OctreeTraversal final : public USVORayCaster
{
    GENERATED_BODY()

protected:

    bool TraceInternal( const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to ) const override;

private:

    struct FOctreeRay
    {
        FOctreeRay( float tx0, float tx1, float ty0, float ty1, float tz0, float tz1 );
        bool Intersects() const;
        bool IsInRange( float max_size ) const;

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

    bool DoesRayIntersectOccludedSubNode( const FOctreeRay & ray, const FSVONodeAddress & node_address, const NodeIndex leaf_sub_node_index, const FSVOVolumeNavigationData & data ) const;
    bool DoesRayIntersectOccludedLeaf( const FOctreeRay & ray, const FSVONodeAddress & node_address, const FSVOVolumeNavigationData & data ) const;
    bool DoesRayIntersectOccludedNormalNode( const FOctreeRay & ray, const FSVONodeAddress & node_address, const FSVOVolumeNavigationData & data ) const;
    bool DoesRayIntersectOccludedNode( const FOctreeRay & ray, const FSVONodeAddress & node_address, const FSVONodeAddress & parent_node_address, const FSVOVolumeNavigationData & data ) const;

    mutable uint8 a = 0;
    mutable float RaySize = 0.0f;
};
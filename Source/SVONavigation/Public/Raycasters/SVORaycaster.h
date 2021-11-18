#pragma once

#include "SVONavigationTypes.h"

#include <CoreMinimal.h>

#include "SVORaycaster.generated.h"

struct FSVORaycasterTraversedNode
{
    FSVORaycasterTraversedNode() = default;
    FSVORaycasterTraversedNode( const FSVONodeAddress & node_address, const bool is_occluded ) :
        NodeAddress( node_address ),
        bIsOccluded( is_occluded )
    {
    }

    FSVONodeAddress NodeAddress;
    bool bIsOccluded;
};

UCLASS( Abstract, NotBlueprintable, EditInlineNew )
class SVONAVIGATION_API USVORaycaster : public UObject
{
    GENERATED_BODY()

public:

    bool HasLineOfSight( UObject * world_context, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const;
    bool HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FSVONodeAddress from, const FSVONodeAddress to, const FNavAgentProperties & nav_agent_properties ) const;
    bool HasLineOfSight( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const;

    const TArray< FSVORaycasterTraversedNode > & GetTraversedNodes() const;

protected:

    virtual bool HasLineOfSightInternal( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FSVONodeAddress from, const FSVONodeAddress to, const FNavAgentProperties & nav_agent_properties ) const;
    virtual bool HasLineOfSightInternal( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const;

    mutable TArray< FSVORaycasterTraversedNode > TraversedNodes;
};

FORCEINLINE const TArray< FSVORaycasterTraversedNode > & USVORaycaster::GetTraversedNodes() const
{
    return TraversedNodes;
}
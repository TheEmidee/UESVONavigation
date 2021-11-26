#pragma once

#include "SVONavigationTypes.h"

#include <CoreMinimal.h>

#include "SVORayCaster.generated.h"

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

struct FSVORayCasterDebugInfos
{
    FVector RayCastStartLocation;
    FVector RayCastEndLocation;
    bool Result;
    TArray< FSVORaycasterTraversedNode > TraversedNodes;
    TArray< FSVORaycasterTraversedNode > TraversedLeafNodes;
    TArray< FSVORaycasterTraversedNode > TraversedLeafSubNodes;
    const FSVOVolumeNavigationData * NavigationData;
};

class FSVORayCasterObserver
{
public:
    virtual ~FSVORayCasterObserver() = default;

    virtual void Initialize( const FSVOVolumeNavigationData * navigation_data, const FVector from, const FVector to ) {}
    virtual void SetResult( bool result ) {}
    virtual void AddTraversedNode( FSVONodeAddress node_address, bool is_occluded ) {}
    virtual void AddTraversedLeafSubNode( FSVONodeAddress node_address, bool is_occluded ) {}
};

class FSVORayCasterObserver_GenerateDebugInfos final : public FSVORayCasterObserver
{
public:
    explicit FSVORayCasterObserver_GenerateDebugInfos( FSVORayCasterDebugInfos & debug_infos );

    void Initialize( const FSVOVolumeNavigationData * navigation_data, const FVector from, const FVector to ) override;
    void SetResult( bool result ) override;
    void AddTraversedNode( FSVONodeAddress node_address, bool is_occluded ) override;
    void AddTraversedLeafSubNode( FSVONodeAddress node_address, bool is_occluded ) override;

private:
    FSVORayCasterDebugInfos & DebugInfos;
};

UCLASS( Abstract, NotBlueprintable, EditInlineNew )
class SVONAVIGATION_API USVORayCaster : public UObject
{
    GENERATED_BODY()

public:
    bool Trace( const FSVOVolumeNavigationData & volume_navigation_data, const FSVONodeAddress from, const FSVONodeAddress to ) const;
    bool Trace( const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to ) const;

    void SetObserver( TSharedPtr< FSVORayCasterObserver > observer );

protected:
    virtual bool TraceInternal( const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to ) const;

    TSharedPtr< FSVORayCasterObserver > Observer;
};
#pragma once

#include "SVONavigationPath.h"
#include "SVONavigationTypes.h"

#include "SVOPathFindingAlgorithmTypes.generated.h"

struct FSVONavigationQueryFilterSettings;
class FSVONavigationQueryFilterImpl;

struct FSVONodeAddressWithLocation
{
    FSVONodeAddressWithLocation() = default;
    FSVONodeAddressWithLocation( const FSVONodeAddress & node_address, const FVector & location );
    FSVONodeAddressWithLocation( const FSVONodeAddress & node_address, const FSVOVolumeNavigationData & bounds_navigation_data );

    bool operator==( const FSVONodeAddressWithLocation & other ) const
    {
        return NodeAddress == other.NodeAddress;
    }

    bool operator!=( const FSVONodeAddressWithLocation & other ) const
    {
        return !operator==( other );
    }

    FSVONodeAddress NodeAddress;
    FVector Location;
};

FORCEINLINE uint32 GetTypeHash( const FSVONodeAddressWithLocation & node_address_with_location )
{
    return GetTypeHash( node_address_with_location.NodeAddress );
}

struct FSVOPathFinderDebugNodeCost
{
    FSVOPathFinderDebugNodeCost() = default;
    FSVOPathFinderDebugNodeCost( const FSVONodeAddressWithLocation & from, const FSVONodeAddressWithLocation & to, const float cost, const bool is_closed );

    void Reset();

    FSVONodeAddressWithLocation From;
    FSVONodeAddressWithLocation To;
    double Cost;
    uint8 bIsClosed : 1;
};

struct FSVOPathFinderNodeAddressWithCost
{
    FSVOPathFinderNodeAddressWithCost() = default;
    FSVOPathFinderNodeAddressWithCost( const FSVONodeAddress & node_address, double traversal_cost );

    FSVONodeAddress NodeAddress;
    double Cost;
};

USTRUCT()
struct SVONAVIGATION_API FSVOPathFinderDebugInfos
{
    GENERATED_USTRUCT_BODY()

    void Reset();

    FSVOPathFinderDebugNodeCost LastProcessedSingleNode;
    TArray< FSVOPathFinderDebugNodeCost > ProcessedNeighbors;

    FSVONavigationPath CurrentBestPath;

    UPROPERTY( VisibleAnywhere )
    int Iterations;

    UPROPERTY( VisibleAnywhere )
    int VisitedNodes;

    UPROPERTY( VisibleAnywhere )
    int PathSegmentCount;

    UPROPERTY( VisibleAnywhere )
    float PathLength;

    UPROPERTY( VisibleAnywhere )
    FString StartNodeAddress;

    UPROPERTY( VisibleAnywhere )
    FString EndNodeAddress;
};

struct FSVOPathFindingParameters
{
    static TOptional< FSVOPathFindingParameters > Initialize( const FSVOVolumeNavigationData & volume_navigation_data, const FVector & start_location, const FVector & end_location, const FNavigationQueryFilter & nav_query_filter );

    FVector StartLocation;
    FVector EndLocation;
    const FNavigationQueryFilter & NavigationQueryFilter;
    const FSVONavigationQueryFilterImpl * QueryFilterImplementation;
    const FSVONavigationQueryFilterSettings & QueryFilterSettings;
    const USVOPathHeuristicCalculator * HeuristicCalculator;
    const USVOPathTraversalCostCalculator * CostCalculator;
    const FSVOVolumeNavigationData & VolumeNavigationData;
    FSVONodeAddress StartNodeAddress;
    FSVONodeAddress EndNodeAddress;

private:
    FSVOPathFindingParameters( const FSVOVolumeNavigationData & volume_navigation_data, const FVector & start_location, const FVector & end_location, const FNavigationQueryFilter & nav_query_filter );
};
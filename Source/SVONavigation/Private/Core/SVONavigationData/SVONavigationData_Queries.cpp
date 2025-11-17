#include "SVONavigationData.h"
#include "SVOVolumeNavigationData.h"
#include "PathFinding/SVONavigationQueryFilterImpl.h"
#include "PathFinding/SVOPathFinder.h"
#include "NavMesh/NavMeshPath.h"
#include "PathFinding/SVONavigationPath.h"

FNavLocation ASVONavigationData::GetRandomPoint( FSharedConstNavQueryFilter /*filter*/, const UObject * /*querier*/ ) const
{
    FNavLocation result;

    const auto navigation_bounds_num = VolumeNavigationData.Num();

    if ( navigation_bounds_num == 0 )
    {
        return result;
    }

    TArray< int > navigation_bounds_indices;
    navigation_bounds_indices.Reserve( VolumeNavigationData.Num() );

    for ( auto index = 0; index < navigation_bounds_num; index++ )
    {
        navigation_bounds_indices.Add( index );
    }

    // Shuffle the array
    for ( int index = navigation_bounds_indices.Num() - 1; index > 0; --index )
    {
        const auto new_index = FMath::RandRange( 0, index );
        Swap( navigation_bounds_indices[ index ], navigation_bounds_indices[ new_index ] );
    }

    do
    {
        const auto index = navigation_bounds_indices.Pop( EAllowShrinking::No );
        const auto & volume_navigation_data = VolumeNavigationData[ index ];

        const auto random_point = volume_navigation_data.GetRandomPoint();
        if ( random_point.IsSet() )
        {
            result = random_point.GetValue();
            break;
        }
    } while ( navigation_bounds_indices.Num() > 0 );

    return result;
}

bool ASVONavigationData::GetRandomReachablePointInRadius( const FVector & origin, float radius, FNavLocation & out_result, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    // :TODO:
    ensure( false );
    return false;
}

bool ASVONavigationData::GetRandomPointInNavigableRadius( const FVector & origin, float Radius, FNavLocation & out_result, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    // :TODO:
    ensure( false );
    return false;
}

void ASVONavigationData::BatchRaycast( TArray< FNavigationRaycastWork > & workload, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    // :TODO:
    ensure( false );
}

bool ASVONavigationData::FindMoveAlongSurface( const FNavLocation & start_location, const FVector & target_position, FNavLocation & out_location, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    // :TODO:
    ensure( false );
    return false;
}

bool ASVONavigationData::ProjectPoint( const FVector & point, FNavLocation & out_location, const FVector & extent, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    if (VolumeNavigationData.IsEmpty())
    {
        return false;
    }

    // 1. Find the correct volume to search in.
    const FSVOVolumeNavigationData* VolumeToSearch = nullptr;
    for (const auto& Volume : VolumeNavigationData)
    {
        if (Volume.GetData().GetNavigationBounds().IsInside(point))
        {
            VolumeToSearch = &Volume;
            break;
        }
    }

    // Fallback: If the point is outside all volumes, find the closest volume to search within.
    if (!VolumeToSearch)
    {
        float MinDistSq = -1.0f;
        for (const auto& Volume : VolumeNavigationData)
        {
            const FBox& BoundingBox = Volume.GetData().GetNavigationBounds();
            if (!BoundingBox.IsValid)
                continue;

            const float DistSq = BoundingBox.ComputeSquaredDistanceToPoint(point);
            if (MinDistSq < 0 || DistSq < MinDistSq)
            {
                MinDistSq = DistSq;
                VolumeToSearch = &Volume;
            }
        }
    }

    if ( !VolumeToSearch )
    {
        return false;
    }

    // 2. Clamp the search point to be within the volume's bounds.
    FVector StartPoint = point;
    const FBox& VolumeBounds = VolumeToSearch->GetData().GetNavigationBounds();
    if (!VolumeBounds.IsInside(StartPoint))
    {
        StartPoint = VolumeBounds.GetClosestPointTo(StartPoint);
    }

    // 3. Attempt to find the initial node and check if it's already navigable.
    FSVONodeAddress InitialAddress;
    const bool bInitialNodeFound = VolumeToSearch->GetNodeAddressFromPosition(InitialAddress, StartPoint);
    if (bInitialNodeFound && VolumeToSearch->IsNodeAddressNavigable(InitialAddress))
    {
        out_location.Location = StartPoint;
        out_location.NodeRef = InitialAddress.GetNavNodeRef();
        return true;
    }

    // 4. BFS Initialization: The start point is either in an occluded node or couldn't be resolved.
    //    We must perform a search for the nearest navigable one.
    TQueue<FSVONodeAddress> OpenList;
    TSet<FSVONodeAddress> VisitedList;
    const FBox SearchBounds = FBox::BuildAABB(StartPoint, extent);
    
    if (bInitialNodeFound)
    {
        // Start point is in an occluded node, begin search from there.
        OpenList.Enqueue(InitialAddress);
        VisitedList.Add(InitialAddress);
    }
    else
    {
        // The start point could not be resolved to any node.
        // Seed the search with the nearest nodes to the start point instead of failing.
        
        TArray<FSVONodeAddress> SeedNodes;
        // A small radius, just enough to find the immediate surrounding nodes.
        const float SeedRadius = VolumeToSearch->GetData().GetLeafNodes().GetLeafNodeExtent() * 1.5f;
        VolumeToSearch->FindNodesInSphere(StartPoint, SeedRadius, SeedNodes);
        
        if (SeedNodes.IsEmpty())
        {
             return false;
        }

        for (const FSVONodeAddress& SeedNode : SeedNodes)
        {
            if (!VisitedList.Contains(SeedNode))
            {
                 OpenList.Enqueue(SeedNode);
                 VisitedList.Add(SeedNode);
            }
        }
    }
    
    // 5. BFS Loop
    FSVONodeAddress CurrentAddress;
    while ( OpenList.Dequeue(CurrentAddress))
    {
        TArray<FSVONodeAddress> Neighbors;
        VolumeToSearch->GetNodeNeighbors(Neighbors, CurrentAddress);

        for (const FSVONodeAddress& NeighborAddress : Neighbors)
        {
            if (!VisitedList.Contains( NeighborAddress))
            {
                VisitedList.Add(NeighborAddress);

                const FVector NeighborLocation = VolumeToSearch->GetNodePositionFromAddress(NeighborAddress, true);

                if (!SearchBounds.IsInside(NeighborLocation))
                {
                    continue;
                }

                if (VolumeToSearch->IsNodeAddressNavigable(NeighborAddress))
                {
                    out_location.Location = NeighborLocation;
                    out_location.NodeRef = NeighborAddress.GetNavNodeRef();
                    return true;
                }

                OpenList.Enqueue(NeighborAddress);
            }
        }
    }

    // 6. Failure
    return false;
}

void ASVONavigationData::BatchProjectPoints( TArray< FNavigationProjectionWork > & Workload, const FVector & Extent, FSharedConstNavQueryFilter Filter, const UObject * Querier ) const
{
    for (FNavigationProjectionWork& WorkItem : Workload)
    {
        WorkItem.bResult = ProjectPoint(WorkItem.Point, WorkItem.OutLocation, Extent, Filter, Querier);
    }
}

void ASVONavigationData::BatchProjectPoints( TArray< FNavigationProjectionWork > & Workload, FSharedConstNavQueryFilter Filter, const UObject * Querier ) const
{
    for (FNavigationProjectionWork& WorkItem : Workload)
    {
        WorkItem.bResult = ProjectPoint(WorkItem.Point, WorkItem.OutLocation, FVector::ZeroVector, Filter, Querier);
    }
}

ENavigationQueryResult::Type ASVONavigationData::CalcPathCost( const FVector & path_start, const FVector & path_end, FVector::FReal & out_path_cost, const FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    FVector::FReal path_length = 0.f;
    return CalcPathLengthAndCost( path_start, path_end, path_length, out_path_cost, filter, querier );
}

ENavigationQueryResult::Type ASVONavigationData::CalcPathLength( const FVector & path_start, const FVector & path_end, FVector::FReal & out_path_length, const FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    FVector::FReal path_cost = 0.f;
    return CalcPathLengthAndCost( path_start, path_end, out_path_length, path_cost, filter, querier );
}

ENavigationQueryResult::Type ASVONavigationData::CalcPathLengthAndCost( const FVector & path_start, const FVector & path_end, FVector::FReal & out_path_length, FVector::FReal & out_path_cost, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    ENavigationQueryResult::Type result = ENavigationQueryResult::Invalid;

    if ( ( path_start - path_end ).IsNearlyZero() )
    {
        out_path_length = 0.f;
        return ENavigationQueryResult::Success;
    }

    auto * volume_navigation_data = GetVolumeNavigationDataContainingPoints( { path_start, path_end } );

    if ( volume_navigation_data == nullptr )
    {
        return ENavigationQueryResult::Error;
    }

    const TSharedRef< FSVONavigationPath > navigation_path = MakeShareable( new FSVONavigationPath() );

    result = FSVOPathFinder::GetPath( navigation_path.Get(), *this, path_start, path_end, filter );

    if ( result == ENavigationQueryResult::Success || ( result == ENavigationQueryResult::Fail && navigation_path->IsPartial() ) )
    {
        out_path_length = navigation_path->GetLength();
        out_path_cost = navigation_path->GetCost();
    }

    return result;
}

bool ASVONavigationData::DoesNodeContainLocation( NavNodeRef node_ref, const FVector & world_space_location ) const
{
    const FSVONodeAddress Address(node_ref);
    if (!Address.IsValid())
    {
        return false;
    }

    for (const auto& Volume : VolumeNavigationData)
    {
        // A simple check to see if the location is even in this volume. This isn't perfect
        // as a node from one volume could technically contain a point just inside another,
        // but it's a reasonable optimization.
        if (Volume.GetData().GetNavigationBounds().IsInside(world_space_location))
        {
            const FVector NodeLocation = Volume.GetNodePositionFromAddress(Address, true);
            const float NodeExtent = Volume.GetNodeExtentFromNodeAddress(Address);
            const FBox NodeBounds = FBox::BuildAABB(NodeLocation, FVector(NodeExtent));

            if (NodeBounds.IsInsideOrOn(world_space_location))
            {
                return true;
            }
        }
    }

    return false;
}

bool ASVONavigationData::IsNodeRefValid( const NavNodeRef node_ref ) const
{
    return FSVONodeAddress( node_ref ).IsValid();
}

FBox ASVONavigationData::GetBoundingBox() const
{
    FBox bounding_box( ForceInit );

    for ( const auto & bounds : VolumeNavigationData )
    {
        bounding_box += bounds.GetData().GetNavigationBounds();
    }

    return bounding_box;
}

const FSVOVolumeNavigationData * ASVONavigationData::GetVolumeNavigationDataContainingPoints( const TArray< FVector > & points ) const
{
    return VolumeNavigationData.FindByPredicate( [ this, &points ]( const FSVOVolumeNavigationData & data ) {
        const auto & bounds = data.GetData().GetNavigationBounds();
        for ( const auto & point : points )
        {
            if ( !bounds.IsInside( point ) )
            {
                return false;
            }
        }
        return true;
    } );
}
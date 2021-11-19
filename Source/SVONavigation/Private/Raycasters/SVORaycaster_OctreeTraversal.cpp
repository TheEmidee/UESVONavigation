#include "Raycasters/SVORaycaster_OctreeTraversal.h"

#include "SVOVolumeNavigationData.h"

#include <DrawDebugHelpers.h>

/* This is an implementation of An Efficient Parametric Algorithm for Octree Traversal : http://wscg.zcu.cz/wscg2000/Papers_2000/X31.pdf
Some code examples :
https://github.com/kwstanths/Ray-traversal/blob/master/TrianglesOctree.hpp
https://newbedev.com/ray-octree-intersection-algorithms

But because we use morton codes to store the node coordinates, the original algorithm needs to be updated because the order is different

Node order in the paper
Y
^
|
|
|          3 - 7
|        /   / |
|  Z    2 - 6  5
| /     |   | /
|/      0 - 4
+-------------------> X

Node order with morton codes
Y
^
|
|
|          6 - 7
|        /   / |
|  Z    2 - 3  5
| /     |   | /
|/      0 - 1
+-------------------> X

*/

bool USVORayCaster_OctreeTraversal::TraceInternal( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const
{
    const auto & volume_bounds = volume_navigation_data.GetVolumeBounds();
    FVector volume_center;
    FVector volume_half_extent;
    volume_bounds.GetCenterAndExtents( volume_center, volume_half_extent );

    FRay ray( from, ( to - from ) );

    a = 0;

    if ( ray.Direction.X < 0.0f )
    {
        ray.Origin.X = volume_center.X * 2 - ray.Origin.X;
        ray.Direction.X = -ray.Direction.X;
        a |= 1;
    }

    if ( ray.Direction.Y < 0.0f )
    {
        ray.Origin.Y = volume_center.Y * 2.0f - ray.Origin.Y;
        ray.Direction.Y = -ray.Direction.Y;
        a |= 2;
    }

    if ( ray.Direction.Z < 0.0f )
    {
        ray.Origin.Z = volume_center.Z * 2.0f - ray.Origin.Z;
        ray.Direction.Z = -ray.Direction.Z;
        a |= 4;
    }

    const auto div_x = 1.0f / ray.Direction.X;
    const auto div_y = 1.0f / ray.Direction.Y;
    const auto div_z = 1.0f / ray.Direction.Z;

    const FOctreeRay octree_ray(
        ( volume_bounds.Min.X - ray.Origin.X ) * div_x,
        ( volume_bounds.Max.X - ray.Origin.X ) * div_x,
        ( volume_bounds.Min.Y - ray.Origin.Y ) * div_y,
        ( volume_bounds.Max.Y - ray.Origin.Y ) * div_y,
        ( volume_bounds.Min.Z - ray.Origin.Z ) * div_z,
        ( volume_bounds.Max.Z - ray.Origin.Z ) * div_z );

    //World = world_context->GetWorld();

    UE_LOG( LogTemp, Warning, TEXT( "USVORayCaster_OctreeTraversal" ) );

    /*FlushPersistentDebugLines( World );
    DrawDebugLine( World, from, to, FColor::Magenta, true, 0.5f, 0, 5.0f );*/

    if ( !octree_ray.Intersects() )
    {
        return true;
    }

    return DoesRayIntersectOccludedNode( octree_ray, FSVONodeAddress( volume_navigation_data.GetData().GetLayerCount() - 1, 0 ), FSVONodeAddress::InvalidAddress, volume_navigation_data );
}

USVORayCaster_OctreeTraversal::FOctreeRay::FOctreeRay( const float tx0, const float tx1, const float ty0, const float ty1, const float tz0, const float tz1 ) :
    tx0( tx0 ),
    tx1( tx1 ),
    txm( 0.5f * ( tx0 + tx1 ) ),
    ty0( ty0 ),
    ty1( ty1 ),
    tym( 0.5f * ( ty0 + ty1 ) ),
    tz0( tz0 ),
    tz1( tz1 ),
    tzm( 0.5f * ( tz0 + tz1 ) )
{
}

bool USVORayCaster_OctreeTraversal::FOctreeRay::Intersects() const
{
    return FMath::Max3( tx0, ty0, tz0 ) < FMath::Min3( tx1, ty1, tz1 );
}

uint8 USVORayCaster_OctreeTraversal::GetFirstNodeIndex( const FOctreeRay & ray )
{
    uint8 answer = 0;

    // select the entry plane and set bits ( cf Table 1 and 2 of the paper)
    // Updated to match morton coords ordering
    if ( ray.tx0 > ray.ty0 )
    {
        if ( ray.tx0 > ray.tz0 )
        {
            if ( ray.ty1 < ray.tx0 )
            {
                answer |= 2;
            }
            if ( ray.tz1 < ray.tx0 )
            {
                answer |= 4;
            }
            return answer;
        }
    }
    else
    {
        if ( ray.ty0 > ray.tz0 )
        {
            if ( ray.tx1 < ray.ty0 )
            {
                answer |= 1;
            }
            if ( ray.tz1 < ray.ty0 )
            {
                answer |= 4;
            }
            return answer;
        }
    }

    if ( ray.tx1 < ray.tz0 )
    {
        answer |= 1;
    }
    if ( ray.ty1 < ray.tz0 )
    {
        answer |= 2;
    }
    return answer;
}

uint8 USVORayCaster_OctreeTraversal::GetNextNodeIndex( const float txm, const int32 x, const float tym, const int32 y, const float tzm, const int32 z )
{
    if ( txm < tym )
    {
        if ( txm < tzm )
        {
            return x;
        }
    }
    else
    {
        if ( tym < tzm )
        {
            return y;
        }
    }

    return z;
}

bool USVORayCaster_OctreeTraversal::DoesRayIntersectOccludedSubNode( const FOctreeRay & ray, const FSVONodeAddress & node_address, const FSVOVolumeNavigationData & data ) const
{
    return false;
}

bool USVORayCaster_OctreeTraversal::DoesRayIntersectOccludedLeaf( const FOctreeRay & ray, const FSVONodeAddress & node_address, const FSVONodeAddress & parent_node_address, const FSVOVolumeNavigationData & data ) const
{
    const auto node_index = node_address.NodeIndex;
    const FSVOLeaf & leaf_node = data.GetData().GetLeaves().GetLeaf( node_index );

    UE_LOG( LogTemp, Warning, TEXT( "LeafNode Address : 0 %i" ), node_address.NodeIndex );

    if ( leaf_node.IsCompletelyFree() )
    {
        return false;
    }

    return false;
}

bool USVORayCaster_OctreeTraversal::DoesRayIntersectOccludedNormalNode( const FOctreeRay & ray, const FSVONodeAddress & node_address, const FSVONodeAddress & parent_node_address, const FSVOVolumeNavigationData & data ) const
{
    const auto & node = data.GetData().GetLayer( node_address.LayerIndex ).GetNode( node_address.NodeIndex );

#if WITH_EDITOR
    UE_LOG( LogTemp, Warning, TEXT( "Node Address : %i - %i - %i" ), node_address.LayerIndex, node_address.NodeIndex, node_address.SubNodeIndex );
#endif

    if ( !node.HasChildren() )
    {
        return false;
    }

    const auto & first_child_address = node.FirstChild;
    auto child_index = GetFirstNodeIndex( FOctreeRay( ray.tx0, ray.txm, ray.ty0, ray.tym, ray.tz0, ray.tzm ) );

    do
    {
        const auto child_node_index = first_child_address.NodeIndex + ( child_index ^ a );
        const FSVONodeAddress new_child_address( first_child_address.LayerIndex, child_node_index );
        switch ( child_index )
        {
            case 0:
            {
                if ( DoesRayIntersectOccludedNode( FOctreeRay( ray.tx0, ray.txm, ray.ty0, ray.tym, ray.tz0, ray.tzm ), new_child_address, node_address, data ) )
                {
                    return true;
                }
                child_index = GetNextNodeIndex( ray.txm, 1, ray.tym, 2, ray.tzm, 4 );
            }
            break;
            case 1:
            {
                if ( DoesRayIntersectOccludedNode( FOctreeRay( ray.txm, ray.tx1, ray.ty0, ray.tym, ray.tz0, ray.tzm ), new_child_address, node_address, data ) )
                {
                    return true;
                }
                child_index = GetNextNodeIndex( ray.tx1, 8, ray.tym, 3, ray.tzm, 5 );
            }
            break;
            case 2:
            {
                if ( DoesRayIntersectOccludedNode( FOctreeRay( ray.tx0, ray.txm, ray.tym, ray.ty1, ray.tz0, ray.tzm ), new_child_address, node_address, data ) )
                {
                    return true;
                }
                child_index = GetNextNodeIndex( ray.txm, 3, ray.ty1, 8, ray.tzm, 6 );
            }
            break;
            case 3:
            {
                if ( DoesRayIntersectOccludedNode( FOctreeRay( ray.txm, ray.tx1, ray.tym, ray.ty1, ray.tz0, ray.tzm ), new_child_address, node_address, data ) )
                {
                    return true;
                }
                child_index = GetNextNodeIndex( ray.tx1, 8, ray.ty1, 8, ray.tzm, 7 );
            }
            break;
            case 4:
            {
                if ( DoesRayIntersectOccludedNode( FOctreeRay( ray.tx0, ray.txm, ray.ty0, ray.tym, ray.tzm, ray.tz1 ), new_child_address, node_address, data ) )
                {
                    return true;
                }
                child_index = GetNextNodeIndex( ray.txm, 5, ray.tym, 6, ray.tz1, 8 );
            }
            break;
            case 5:
            {
                if ( DoesRayIntersectOccludedNode( FOctreeRay( ray.txm, ray.tx1, ray.ty0, ray.tym, ray.tzm, ray.tz1 ), new_child_address, node_address, data ) )
                {
                    return true;
                }
                child_index = GetNextNodeIndex( ray.tx1, 8, ray.tym, 7, ray.tz1, 8 );
            }
            break;
            case 6:
            {
                if ( DoesRayIntersectOccludedNode( FOctreeRay( ray.tx0, ray.txm, ray.tym, ray.ty1, ray.tzm, ray.tz1 ), new_child_address, node_address, data ) )
                {
                    return true;
                }
                child_index = GetNextNodeIndex( ray.txm, 7, ray.ty1, 8, ray.tz1, 8 );
            }
            break;
            case 7:
            {
                if ( DoesRayIntersectOccludedNode( FOctreeRay( ray.txm, ray.tx1, ray.tym, ray.ty1, ray.tzm, ray.tz1 ), new_child_address, node_address, data ) )
                {
                    return true;
                }
                child_index = 8;
            }
            break;
            default:
            {
                checkNoEntry();
                child_index = 8;
            }
            break;
        }
    } while ( child_index < 8 );

    return false;
}

bool USVORayCaster_OctreeTraversal::DoesRayIntersectOccludedNode( const FOctreeRay & ray, const FSVONodeAddress & node_address, const FSVONodeAddress & parent_node_address, const FSVOVolumeNavigationData & data ) const
{
    if ( ray.tx1 < 0.0f || ray.ty1 < 0.0f || ray.tz1 < 0.0f )
    {
        return false;
    }

    const auto layer_index = node_address.LayerIndex;
    bool result;

    if ( layer_index == 0 )
    {
        result = DoesRayIntersectOccludedLeaf( ray, node_address, parent_node_address, data );

        if ( Observer.IsValid() )
        {
            Observer->AddTraversedLeafNode( node_address, result );
        }
    }
    else
    {
        result = DoesRayIntersectOccludedNormalNode( ray, node_address, parent_node_address, data );

        if ( Observer.IsValid() )
        {
            Observer->AddTraversedNode( node_address, result );
        }
    }

    return result;
}

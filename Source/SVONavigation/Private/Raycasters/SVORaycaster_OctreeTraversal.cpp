#include "Raycasters/SVORaycaster_OctreeTraversal.h"

#include "SVOHelpers.h"
#include "SVOVolumeNavigationData.h"

/* This is an implementation of An Efficient Parametric Algorithm for Octree Traversal : http://wscg.zcu.cz/wscg2000/Papers_2000/X31.pdf
Some code examples :
https://github.com/kwstanths/Ray-traversal/blob/master/TrianglesOctree.hpp
https://newbedev.com/ray-octree-intersection-algorithms

But because we use morton codes to store the node coordinates, the original algorithm needs to be updated because the order is different.
This is why the function GetFirstNodeIndex returns a different index, and why the various parameters passed to GetNextNodeIndex or when creating FOctreeRay are not stricly the same as in the paper.

Node ordering in the paper:

Z
^
|          5 --- 7
|        / |   / |
|       1 --- 3  |
|  X    |  4 -|- 6
| /     | /   | /
|/      0 --- 2
+-------------------> Y

Node ordering with morton codes:

Z
^
|          5 --- 7
|        / |   / |
|       4 --- 6  |
|  X    |  1 -|- 3
| /     | /   | /
|/      0 --- 2
+-------------------> Y







*****************************************
***  How sub node intersection works  ***
*****************************************

Leaf nodes are split in 4 cubes in each direction (64 cubes in total), but the data is contained in a single 64 bit uint.

Here is the sub node ordering on the (Y;Z) axis (which will be used to explain below)

Z
|
|   36 38 52 54
|   32 34 48 50
|   04 06 20 22
|   00 02 16 18
|
------------------ Y

Finding the sub node occlusion is done in 2 parts:

1. In DoesRayIntersectOccludedLeaf, instead of working with 16 cubes for each face, work with 4 cubes as with normal nodes. The index of the cube being processed corresponds to one of those 4 cubes.
For example, if the ray goes through the lower right half of the leaf (with axes being the same as in the schema above), the child index will be 2.

Z
|
|   04 06
|   00 02
|
-------------- Y

2. Pass that node index to DoesRayIntersectOccludedSubNode which will split the cube again in 4 to access the sub node.
If for example the ray intersects the upper left sub node of the previously hit cube, GetFirstNodeIndex would return 4.
We are finally able to get the sub node index using the formula : ( SubNodeParentNodeIndex << 3 ) + (ChildIndex)
which in this case returns 20 as expected.
From there we can apply the rules of the algorithm to know which of the neighbor sub nodes to test.
*/

bool USVORayCaster_OctreeTraversal::TraceInternal( UObject * world_context, const FSVOVolumeNavigationData & volume_navigation_data, const FVector & from, const FVector & to, const FNavAgentProperties & nav_agent_properties ) const
{
    const auto & navigation_bounds = volume_navigation_data.GetNavigationBounds();
    FVector volume_center;
    FVector volume_extent;
    navigation_bounds.GetCenterAndExtents( volume_center, volume_extent );

    const FVector from_to( to - from );
    FRay ray( from, from_to );

    a = 0;

    if ( FMath::IsNearlyZero( ray.Direction.X ) )
    {
        ray.Direction.X = SMALL_NUMBER;
    }

    if ( FMath::IsNearlyZero( ray.Direction.Y ) )
    {
        ray.Direction.Y = SMALL_NUMBER;
    }

    if ( FMath::IsNearlyZero( ray.Direction.Z ) )
    {
        ray.Direction.Z = SMALL_NUMBER;
    }

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
        ( navigation_bounds.Min.X - ray.Origin.X ) * div_x,
        ( navigation_bounds.Max.X - ray.Origin.X ) * div_x,
        ( navigation_bounds.Min.Y - ray.Origin.Y ) * div_y,
        ( navigation_bounds.Max.Y - ray.Origin.Y ) * div_y,
        ( navigation_bounds.Min.Z - ray.Origin.Z ) * div_z,
        ( navigation_bounds.Max.Z - ray.Origin.Z ) * div_z );

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

bool USVORayCaster_OctreeTraversal::FOctreeRay::IsInRange() const
{
    return tx1 >= 0.0f && ty1 >= 0.0f && tz1 >= 0.0f;
}

uint8 USVORayCaster_OctreeTraversal::GetFirstNodeIndex( const FOctreeRay & ray )
{
    uint8 answer = 0;

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

bool USVORayCaster_OctreeTraversal::DoesRayIntersectOccludedSubNode( const FOctreeRay & ray, const FSVONodeAddress & node_address, const NodeIndex leaf_sub_node_index, const FSVOVolumeNavigationData & data ) const
{
    if ( !ray.IsInRange() )
    {
        return false;
    }

    const auto & leaf_node = data.GetData().GetLeafNodes().GetLeafNode( node_address.NodeIndex );
    int32 current_child_idx = GetFirstNodeIndex( FOctreeRay( ray.tx0, ray.txm, ray.ty0, ray.tym, ray.tz0, ray.tzm ) );

    bool result = false;

    do
    {
        // leaf_sub_node_index is the index of one of the 8 cubes of the leaf node (see the explanation above)
        const SubNodeIndex sub_node_idx = ( leaf_sub_node_index << 3 ) + ( current_child_idx ^ a );

        switch ( current_child_idx )
        {
            case 0:
            {
                FOctreeRay sub_node_ray( ray.tx0, ray.txm, ray.ty0, ray.tym, ray.tz0, ray.tzm );
                if ( sub_node_ray.IsInRange() && leaf_node.IsSubNodeOccluded( sub_node_idx ) && sub_node_ray.Intersects() )
                {
                    result = true;
                }
                else
                {
                    current_child_idx = GetNextNodeIndex( ray.txm, 1, ray.tym, 2, ray.tzm, 4 );
                }
                break;
            }
            case 1:
            {
                FOctreeRay sub_node_ray( ray.txm, ray.tx1, ray.ty0, ray.tym, ray.tz0, ray.tzm );
                if ( sub_node_ray.IsInRange() && leaf_node.IsSubNodeOccluded( sub_node_idx ) && sub_node_ray.Intersects() )
                {
                    result = true;
                }
                else
                {
                    current_child_idx = GetNextNodeIndex( ray.tx1, 8, ray.tym, 3, ray.tzm, 5 );
                }
                break;
            }
            case 2:
            {
                FOctreeRay sub_node_ray( ray.tx0, ray.txm, ray.tym, ray.ty1, ray.tz0, ray.tzm );
                if ( sub_node_ray.IsInRange() && leaf_node.IsSubNodeOccluded( sub_node_idx ) && sub_node_ray.Intersects() )
                {
                    result = true;
                }
                else
                {
                    current_child_idx = GetNextNodeIndex( ray.txm, 3, ray.ty1, 8, ray.tzm, 6 );
                }
                break;
            }
            case 3:
            {
                FOctreeRay sub_node_ray( ray.txm, ray.tx1, ray.tym, ray.ty1, ray.tz0, ray.tzm );
                if ( sub_node_ray.IsInRange() && leaf_node.IsSubNodeOccluded( sub_node_idx ) && sub_node_ray.Intersects() )
                {
                    result = true;
                }
                else
                {
                    current_child_idx = GetNextNodeIndex( ray.tx1, 8, ray.ty1, 8, ray.tzm, 7 );
                }
                break;
            }
            case 4:
            {
                FOctreeRay sub_node_ray( ray.tx0, ray.txm, ray.ty0, ray.tym, ray.tzm, ray.tz1 );
                if ( sub_node_ray.IsInRange() && leaf_node.IsSubNodeOccluded( sub_node_idx ) && sub_node_ray.Intersects() )
                {
                    result = true;
                }
                else
                {
                    current_child_idx = GetNextNodeIndex( ray.txm, 5, ray.tym, 6, ray.tz1, 8 );
                }
                break;
            }
            case 5:
            {
                FOctreeRay sub_node_ray( ray.txm, ray.tx1, ray.ty0, ray.tym, ray.tzm, ray.tz1 );
                if ( sub_node_ray.IsInRange() && leaf_node.IsSubNodeOccluded( sub_node_idx ) && sub_node_ray.Intersects() )
                {
                    result = true;
                }
                else
                {
                    current_child_idx = GetNextNodeIndex( ray.tx1, 8, ray.tym, 7, ray.tz1, 8 );
                }
                break;
            }
            case 6:
            {
                FOctreeRay sub_node_ray( ray.tx0, ray.txm, ray.tym, ray.ty1, ray.tzm, ray.tz1 );
                if ( sub_node_ray.IsInRange() && leaf_node.IsSubNodeOccluded( sub_node_idx ) && sub_node_ray.Intersects() )
                {
                    result = true;
                }
                else
                {
                    current_child_idx = GetNextNodeIndex( ray.txm, 7, ray.ty1, 8, ray.tz1, 8 );
                }
                break;
            }
            case 7:
            {
                FOctreeRay sub_node_ray( ray.txm, ray.tx1, ray.tym, ray.ty1, ray.tzm, ray.tz1 );
                if ( sub_node_ray.IsInRange() && leaf_node.IsSubNodeOccluded( sub_node_idx ) && sub_node_ray.Intersects() )
                {
                    result = true;
                }
                else
                {
                    current_child_idx = 8;
                }
                break;
            }
            default:
            {
                checkNoEntry();
                current_child_idx = 8;
            }
            break;
        }

        if ( Observer.IsValid() )
        {
            Observer->AddTraversedLeafSubNode( FSVONodeAddress( 0, node_address.NodeIndex, sub_node_idx ), result );
        }
    } while ( current_child_idx < 8 && !result );

    return result;
}

bool USVORayCaster_OctreeTraversal::DoesRayIntersectOccludedLeaf( const FOctreeRay & ray, const FSVONodeAddress & node_address, const FSVOVolumeNavigationData & data ) const
{
    const auto node_index = node_address.NodeIndex;
    const FSVOLeafNode & leaf_node = data.GetData().GetLeafNodes().GetLeafNode( node_index );

    if ( leaf_node.IsCompletelyFree() )
    {
        return false;
    }

    // Even though leaf nodes don't contain more cubes inside (only 64 sub nodes packed in one 64bit uint) we split that leaf in 2 in all dimensions
    // to find which sub-cube the ray goes in, and then DoesRayIntersectOccludedSubNode does the last split to know which cube of the final resolution is hit
    auto leaf_sub_node_index = GetFirstNodeIndex( FOctreeRay( ray.tx0, ray.txm, ray.ty0, ray.tym, ray.tz0, ray.tzm ) );

    do
    {
        const auto reflected_leaf_sub_node_index = leaf_sub_node_index ^ a;
        switch ( leaf_sub_node_index )
        {
            case 0:
            {
                if ( DoesRayIntersectOccludedSubNode( FOctreeRay( ray.tx0, ray.txm, ray.ty0, ray.tym, ray.tz0, ray.tzm ), node_address, reflected_leaf_sub_node_index, data ) )
                {
                    return true;
                }
                leaf_sub_node_index = GetNextNodeIndex( ray.txm, 1, ray.tym, 2, ray.tzm, 4 );
            }

            break;
            case 1:
            {
                if ( DoesRayIntersectOccludedSubNode( FOctreeRay( ray.txm, ray.tx1, ray.ty0, ray.tym, ray.tz0, ray.tzm ), node_address, reflected_leaf_sub_node_index, data ) )
                {
                    return true;
                }
                leaf_sub_node_index = GetNextNodeIndex( ray.tx1, 8, ray.tym, 3, ray.tzm, 5 );
            }
            break;
            case 2:
            {
                if ( DoesRayIntersectOccludedSubNode( FOctreeRay( ray.tx0, ray.txm, ray.tym, ray.ty1, ray.tz0, ray.tzm ), node_address, reflected_leaf_sub_node_index, data ) )
                {
                    return true;
                }
                leaf_sub_node_index = GetNextNodeIndex( ray.txm, 3, ray.ty1, 8, ray.tzm, 6 );
            }
            break;
            case 3:
            {
                if ( DoesRayIntersectOccludedSubNode( FOctreeRay( ray.txm, ray.tx1, ray.tym, ray.ty1, ray.tz0, ray.tzm ), node_address, reflected_leaf_sub_node_index, data ) )
                {
                    return true;
                }
                leaf_sub_node_index = GetNextNodeIndex( ray.tx1, 8, ray.ty1, 8, ray.tzm, 7 );
            }
            break;
            case 4:
            {
                if ( DoesRayIntersectOccludedSubNode( FOctreeRay( ray.tx0, ray.txm, ray.ty0, ray.tym, ray.tzm, ray.tz1 ), node_address, reflected_leaf_sub_node_index, data ) )
                {
                    return true;
                }
                leaf_sub_node_index = GetNextNodeIndex( ray.txm, 5, ray.tym, 6, ray.tz1, 8 );
            }
            break;
            case 5:
            {
                if ( DoesRayIntersectOccludedSubNode( FOctreeRay( ray.txm, ray.tx1, ray.ty0, ray.tym, ray.tzm, ray.tz1 ), node_address, reflected_leaf_sub_node_index, data ) )
                {
                    return true;
                }
                leaf_sub_node_index = GetNextNodeIndex( ray.tx1, 8, ray.tym, 7, ray.tz1, 8 );
            }
            break;
            case 6:
            {
                if ( DoesRayIntersectOccludedSubNode( FOctreeRay( ray.tx0, ray.txm, ray.tym, ray.ty1, ray.tzm, ray.tz1 ), node_address, reflected_leaf_sub_node_index, data ) )
                {
                    return true;
                }
                leaf_sub_node_index = GetNextNodeIndex( ray.txm, 7, ray.ty1, 8, ray.tz1, 8 );
            }
            break;
            case 7:
            {
                if ( DoesRayIntersectOccludedSubNode( FOctreeRay( ray.txm, ray.tx1, ray.tym, ray.ty1, ray.tzm, ray.tz1 ), node_address, reflected_leaf_sub_node_index, data ) )
                {
                    return true;
                }
                leaf_sub_node_index = 8;
            }
            break;
            default:
            {
                checkNoEntry();
                leaf_sub_node_index = 8;
            }
            break;
        }
    } while ( leaf_sub_node_index < 8 );

    return false;
}

bool USVORayCaster_OctreeTraversal::DoesRayIntersectOccludedNormalNode( const FOctreeRay & ray, const FSVONodeAddress & node_address, const FSVOVolumeNavigationData & data ) const
{
    const auto & node = data.GetData().GetLayer( node_address.LayerIndex ).GetNode( node_address.NodeIndex );

    if ( !node.HasChildren() )
    {
        return false;
    }

    const auto & first_child_address = node.FirstChild;
    auto child_index = GetFirstNodeIndex( FOctreeRay( ray.tx0, ray.txm, ray.ty0, ray.tym, ray.tz0, ray.tzm ) );

    do
    {
        const auto reflected_child_node_index = first_child_address.NodeIndex + ( child_index ^ a );
        const FSVONodeAddress new_child_address( first_child_address.LayerIndex, reflected_child_node_index );
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
    if ( !ray.IsInRange() )
    {
        return false;
    }

    const auto layer_index = node_address.LayerIndex;
    bool result;

    if ( layer_index == 0 )
    {
        result = DoesRayIntersectOccludedLeaf( ray, node_address, data );

        if ( Observer.IsValid() )
        {
            Observer->AddTraversedLeafNode( node_address, result );
        }
    }
    else
    {
        result = DoesRayIntersectOccludedNormalNode( ray, node_address, data );

        if ( Observer.IsValid() )
        {
            Observer->AddTraversedNode( node_address, result );
        }
    }

    return result;
}

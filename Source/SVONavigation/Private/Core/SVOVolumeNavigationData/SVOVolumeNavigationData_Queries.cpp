#include "SVOVolumeNavigationData.h"

#include "SVOHelpers.h"
#include <ThirdParty/libmorton/morton.h>

FVector FSVOVolumeNavigationData::GetNodePositionFromAddress( const FSVONodeAddress & address, const bool try_get_sub_node_position ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetNodePositionFromNodeAddress );

    if ( address.LayerIndex == 0 )
    {
        // Leaf nodes don't have the same NodeIndex as other nodes. They map to the index of the array of leaf nodes.
        // We must then re-construct the leaf node position based on that leaf node parent.
        const auto & leaf_nodes = SVOData.GetLeafNodes();
        const auto & leaf_node = leaf_nodes.GetLeafNode( address.NodeIndex );        
        const auto & leaf_node_parent_node = SVOData.GetLayer( 1 ).GetNode( leaf_node.Parent.NodeIndex );
        
        const auto child_index_offset = address.NodeIndex - leaf_node_parent_node.FirstChild.NodeIndex;
        const auto leaf_node_morton_code = FSVOHelpers::GetFirstChildMortonCode( leaf_node_parent_node.MortonCode ) + child_index_offset;
        const auto leaf_node_extent = leaf_nodes.GetLeafNodeExtent();

        const FVector leaf_node_position = GetLeafNodePositionFromMortonCode( leaf_node_morton_code );

        if ( leaf_node.IsCompletelyFree() || !try_get_sub_node_position )
        {
            return leaf_node_position;
        }

        const auto sub_node_morton_coords = FSVOHelpers::GetVectorFromMortonCode( address.SubNodeIndex );
        const auto sub_node_position = leaf_node_position - leaf_node_extent + sub_node_morton_coords * leaf_nodes.GetLeafSubNodeSize() + leaf_nodes.GetLeafSubNodeExtent();

        return sub_node_position;
    }

    const auto & navigation_bounds = SVOData.GetNavigationBounds();
    const auto navigation_bounds_center = navigation_bounds.GetCenter();
    const auto navigation_bounds_extent = navigation_bounds.GetExtent();

    const auto & layer = SVOData.GetLayer( address.LayerIndex );
    const auto layer_node_size = layer.GetNodeSize();
    const auto layer_node_extent = layer.GetNodeExtent();
    const auto & node = layer.GetNode( address.NodeIndex );
    const auto morton_coords = FSVOHelpers::GetVectorFromMortonCode( node.MortonCode );

    const auto position = navigation_bounds_center - navigation_bounds_extent + morton_coords * layer_node_size + layer_node_extent;

    return position;
}

FVector FSVOVolumeNavigationData::GetNodePositionFromLayerAndMortonCode( const LayerIndex layer_index, const MortonCode morton_code ) const
{
    if ( layer_index == 0 )
    {
        return GetLeafNodePositionFromMortonCode( morton_code );
    }

    const auto & layer = SVOData.GetLayer( layer_index );
    const auto layer_node_extent = layer.GetNodeExtent();
    const auto & navigation_bounds = SVOData.GetNavigationBounds();
    const auto navigation_bounds_center = navigation_bounds.GetCenter();
    const auto navigation_bounds_extent = navigation_bounds.GetExtent();
    const auto layer_node_size = layer.GetNodeSize();
    const auto morton_coords = FSVOHelpers::GetVectorFromMortonCode( morton_code );

    return navigation_bounds_center - navigation_bounds_extent + morton_coords * layer_node_size + layer_node_extent;
}

FVector FSVOVolumeNavigationData::GetLeafNodePositionFromMortonCode( const MortonCode morton_code ) const
{
    const auto & navigation_bounds = SVOData.GetNavigationBounds();
    const auto navigation_bounds_center = navigation_bounds.GetCenter();
    const auto navigation_bounds_extent = navigation_bounds.GetExtent();
    const auto & leaf_nodes = SVOData.GetLeafNodes();
    const auto leaf_node_extent = leaf_nodes.GetLeafNodeExtent();
    const auto leaf_node_size = leaf_nodes.GetLeafNodeSize();
    const auto morton_coords = FSVOHelpers::GetVectorFromMortonCode( morton_code );
    const auto leaf_node_position = navigation_bounds_center - navigation_bounds_extent + morton_coords * leaf_node_size + leaf_node_extent;

    return leaf_node_position;
}

bool FSVOVolumeNavigationData::GetNodeAddressFromPosition( FSVONodeAddress & node_address, const FVector & position ) const
{
    const auto & navigation_bounds = SVOData.GetNavigationBounds();

    if ( !navigation_bounds.IsInside( position ) )
    {
        return false;
    }

    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetNodeAddressFromPosition );

    FVector origin;
    FVector extent;

    navigation_bounds.GetCenterAndExtents( origin, extent );
    // The z-order origin of the volume (where code == 0)
    const auto z_origin = origin - extent;
    // The local position of the point in volume space
    const auto local_position = position - z_origin;

    const auto layer_count = GetLayerCount();
    LayerIndex layer_index = layer_count - 1;
    NodeIndex nodeIndex = 0;

    while ( layer_index >= 0 && layer_index < layer_count )
    {
        const auto & layer = SVOData.GetLayer( layer_index );
        const auto & layer_nodes = layer.GetNodes();
        const auto voxel_size = layer.GetNodeSize();

        FIntVector voxel_coords;
        voxel_coords.X = FMath::FloorToInt( local_position.X / voxel_size );
        voxel_coords.Y = FMath::FloorToInt( local_position.Y / voxel_size );
        voxel_coords.Z = FMath::FloorToInt( local_position.Z / voxel_size );

        // Get the morton code we want for this layer
        const auto code = FSVOHelpers::GetMortonCodeFromVector( voxel_coords );
        const auto node_extent = layer.GetNodeExtent();

        for ( NodeIndex node_index = nodeIndex; node_index < static_cast< uint32 >( layer_nodes.Num() ); node_index++ )
        {
            const auto & node = layer_nodes[ node_index ];

            // This is the node we are in
            if ( node.MortonCode != code )
            {
                continue;
            }

            // There are no child nodes, so this is our nav position
            if ( !node.FirstChild.IsValid() ) // && layerIndex > 0)
            {
                node_address.LayerIndex = layer_index;
                node_address.NodeIndex = node_index;
                node_address.SubNodeIndex = 0;
                return true;
            }

            // If this is a leaf node, we need to find our subnode
            if ( layer_index == 0 )
            {
                const auto & leaf_nodes = SVOData.GetLeafNodes();
                const auto & leaf = leaf_nodes.GetLeafNode( node.FirstChild.NodeIndex );

                // We need to calculate the node local position to get the morton code for the leaf
                // The world position of the 0 node
                const auto node_position = GetLeafNodePositionFromMortonCode( node.MortonCode );
                // The morton origin of the node
                const auto node_origin = node_position - FVector( node_extent );
                // The requested position, relative to the node origin
                const auto node_local_position = position - node_origin;
                // Now get our voxel coordinates
                const auto voxel_quarter_size = voxel_size * 0.25f;

                FIntVector leaf_coords;
                leaf_coords.X = FMath::FloorToInt( node_local_position.X / voxel_quarter_size );
                leaf_coords.Y = FMath::FloorToInt( node_local_position.Y / voxel_quarter_size );
                leaf_coords.Z = FMath::FloorToInt( node_local_position.Z / voxel_quarter_size );

                node_address.LayerIndex = 0;
                node_address.NodeIndex = node_index;

                const auto leaf_code = FSVOHelpers::GetMortonCodeFromVector( leaf_coords ); // This morton code is our key into the 64-bit leaf node

                if ( leaf.IsSubNodeOccluded( leaf_code ) )
                {
                    return false; // This voxel is blocked
                }

                node_address.SubNodeIndex = leaf_code;

                return true;
            }

            // If we've got here, the current node has a child, and isn't a leaf, so lets go down...
            layer_index = layer_nodes[ node_index ].FirstChild.LayerIndex;
            nodeIndex = layer_nodes[ node_index ].FirstChild.NodeIndex;

            break; //stop iterating this layer
        }
    }

    return false;
}

float FSVOVolumeNavigationData::GetLayerRatio( const LayerIndex layer_index ) const
{
    return static_cast< float >( layer_index ) / GetLayerCount();
}

float FSVOVolumeNavigationData::GetLayerInverseRatio( const LayerIndex layer_index ) const
{
    return 1.0f - GetLayerRatio( layer_index );
}

float FSVOVolumeNavigationData::GetNodeExtentFromNodeAddress( const FSVONodeAddress node_address ) const
{
    if ( node_address.LayerIndex == 0 )
    {
        const auto & leaf_nodes = SVOData.GetLeafNodes();
        const auto & leaf_node = leaf_nodes.GetLeafNode( node_address.NodeIndex );
        if ( leaf_node.IsCompletelyFree() )
        {
            return leaf_nodes.GetLeafNodeExtent();
        }

        return leaf_nodes.GetLeafSubNodeExtent();
    }

    return SVOData.GetLayer( node_address.LayerIndex ).GetNodeExtent();
}

TOptional< FNavLocation > FSVOVolumeNavigationData::GetRandomPoint() const
{
    TArray< FSVONodeAddress > non_occluded_nodes;
    const FSVONodeAddress top_most_node_address( GetLayerCount(), 0, 0 );

    GetFreeNodesFromNodeAddress( top_most_node_address, non_occluded_nodes );

    if ( non_occluded_nodes.Num() == 0 )
    {
        return TOptional< FNavLocation >();
    }

    const auto random_index = FMath::RandRange( 0, non_occluded_nodes.Num() - 1 );
    const auto random_node = non_occluded_nodes[ random_index ];
    const auto random_node_location = GetNodePositionFromAddress( random_node, true );
    const auto random_node_extent = GetNodeExtentFromNodeAddress( random_node );

    const auto node_bounds = FBox::BuildAABB( random_node_location, FVector( random_node_extent ) );
    const auto random_point_in_node = FMath::RandPointInBox( node_bounds );
    return FNavLocation( random_point_in_node, random_node.GetNavNodeRef() );
}

bool FSVOVolumeNavigationData::IsNodeAddressNavigable(const FSVONodeAddress& Address) const
{
    if (!Address.IsValid())
    {
        return false;
    }

    const FSVONode& Node = GetNodeFromAddress(Address);

    if (Address.LayerIndex > 0)
    {
        // A non-leaf node is navigable if it represents a large open space (has no children).
        return !Node.HasChildren();
    }
    
    // This is a leaf-level node (Layer 0)
    if (!Node.HasChildren())
    {
        // This is a completely open leaf node.
        return true;
    }

    // This leaf node has sub-nodes. We must check the specific sub-node.
    const FSVOLeafNode& Leaf = SVOData.GetLeafNodes().GetLeafNode(Node.FirstChild.NodeIndex);
    return !Leaf.IsSubNodeOccluded(Address.SubNodeIndex);
}

void FSVOVolumeNavigationData::FindNodesInSphere(const FVector& Center, float Radius, TArray<FSVONodeAddress>& OutNodes) const
{
    if (!SVOData.IsValid())
    {
        return;
    }
    
    const FSVONodeAddress RootNodeAddress(SVOData.GetLayerCount() - 1, 0, 0);
    FindNodesInSphereRecursive(Center, FMath::Square(Radius), RootNodeAddress, OutNodes);
}

void FSVOVolumeNavigationData::GetFreeNodesFromNodeAddress( const FSVONodeAddress node_address, TArray< FSVONodeAddress > & free_nodes ) const
{
    const auto layer_index = node_address.LayerIndex;
    const auto node_index = node_address.NodeIndex;

    if ( layer_index == 0 )
    {
        const auto & leaf_node = SVOData.LeafNodes.GetLeafNode( node_index );

        if ( leaf_node.IsCompletelyOccluded() )
        {
            return;
        }

        if ( leaf_node.IsCompletelyFree() )
        {
            free_nodes.Emplace( node_address );
            return;
        }

        for ( auto morton_code = 0; morton_code < 64; ++morton_code )
        {
            if ( !leaf_node.IsSubNodeOccluded( morton_code ) )
            {
                free_nodes.Emplace( FSVONodeAddress( 0, node_index, morton_code ) );
            }
        }
    }
    else
    {
        const auto & node = SVOData.GetLayer( layer_index ).GetNode( node_index );

        if ( !node.HasChildren() )
        {
            free_nodes.Emplace( node_address );
        }
        else
        {
            const auto & first_child = node.FirstChild;
            const auto child_layer_index = first_child.LayerIndex;
            const auto & child_layer = SVOData.GetLayer( child_layer_index );

            for ( auto child_index = 0; child_index < 8; ++child_index )
            {
                const auto & child_node = child_layer.GetNodes()[ first_child.NodeIndex + child_index ];
                GetFreeNodesFromNodeAddress( FSVONodeAddress( child_layer_index, child_node.MortonCode, 0 ), free_nodes );
            }
        }
    }
}

void FSVOVolumeNavigationData::FindNodesInSphereRecursive(const FVector& Center, float RadiusSq, const FSVONodeAddress& CurrentNodeAddress, TArray<FSVONodeAddress>& OutNodes) const
{
    const FVector NodeCenter = GetNodePositionFromAddress(CurrentNodeAddress, false);
    const float NodeExtent = SVOData.GetLayer(CurrentNodeAddress.LayerIndex).GetNodeExtent();
    const FBox NodeBounds = FBox::BuildAABB(NodeCenter, FVector(NodeExtent));

    if (!FMath::SphereAABBIntersection(FSphere(Center, FMath::Sqrt(RadiusSq)), NodeBounds))
    {
        return;
    }

    const FSVONode& Node = GetNodeFromAddress(CurrentNodeAddress);

    if (!Node.HasChildren())
    {
        OutNodes.AddUnique(CurrentNodeAddress);
        return;
    }

    if (CurrentNodeAddress.LayerIndex > 0)
    {
        const FSVONodeAddress& FirstChildAddress = Node.FirstChild;
        for (uint32 i = 0; i < 8; ++i)
        {
            FSVONodeAddress ChildAddress(FirstChildAddress.LayerIndex, FirstChildAddress.NodeIndex + i, 0);
            FindNodesInSphereRecursive(Center, RadiusSq, ChildAddress, OutNodes);
        }
    }
    else // At Layer 0 with sub-nodes
    {
        const float SubNodeExtent = SVOData.GetLeafNodes().GetLeafSubNodeExtent();
        for (SubNodeIndex i = 0; i < 64; ++i)
        {
            FSVONodeAddress SubNodeAddress(0, CurrentNodeAddress.NodeIndex, i);
            const FVector SubNodeCenter = GetNodePositionFromAddress(SubNodeAddress, true);
            const FBox SubNodeBounds = FBox::BuildAABB(SubNodeCenter, FVector(SubNodeExtent));
            
            if (FMath::SphereAABBIntersection(FSphere(Center, FMath::Sqrt(RadiusSq)), SubNodeBounds))
            {
                OutNodes.AddUnique(SubNodeAddress);
            }
        }
    }
}
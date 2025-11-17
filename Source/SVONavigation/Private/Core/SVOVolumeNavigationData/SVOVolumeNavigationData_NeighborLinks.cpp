#include "SVOVolumeNavigationData.h"
#include "SVOHelpers.h"
#include "Core/SVONavigationConstants.h"

#include <ThirdParty/libmorton/morton.h>

void FSVOVolumeNavigationData::BuildNeighborLinks( const LayerIndex layer_index )
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_BuildNeighborLinks );

    auto & layer_nodes = SVOData.GetLayer( layer_index ).GetNodes();
    const auto max_layer_index = GetLayerCount() - 2;

    for ( NodeIndex layer_node_index = 0; layer_node_index < static_cast< uint32 >( layer_nodes.Num() ); layer_node_index++ )
    {
        auto & node = layer_nodes[ layer_node_index ];

        for ( NeighborDirection direction = 0; direction < 6; direction++ )
        {
            NodeIndex node_index = layer_node_index;
            FSVONodeAddress & neighbor_address = node.Neighbors[ direction ];
            LayerIndex current_layer = layer_index;

            while ( !FindNeighborInDirection( neighbor_address, current_layer, node_index, direction ) && current_layer < max_layer_index )
            {
                auto & parent_address = SVOData.GetLayer( current_layer ).GetNodes()[ node_index ].Parent;
                if ( parent_address.IsValid() )
                {
                    node_index = parent_address.NodeIndex;
                    current_layer = parent_address.LayerIndex;
                }
                else
                {
                    current_layer++;
                    const auto node_index_from_morton = GetNodeIndexFromMortonCode( current_layer, FSVOHelpers::GetParentMortonCode( node.MortonCode ) );
                    check( node_index_from_morton != INDEX_NONE );
                    node_index = static_cast< NodeIndex >( node_index_from_morton );
                }
            }
        }
    }
}

bool FSVOVolumeNavigationData::FindNeighborInDirection( FSVONodeAddress & node_address, const LayerIndex layer_index, const NodeIndex node_index, const NeighborDirection direction )
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_FindNeighborInDirection );

    const auto max_coordinates = static_cast< int32 >( SVOData.GetLayer( layer_index ).GetMaxNodeCount() );
    const auto & layer_nodes = SVOData.GetLayer( layer_index ).GetNodes();
    const auto layer_nodes_count = layer_nodes.Num();
    const auto & target_node = layer_nodes[ node_index ];

    FIntVector neighbor_coords( FSVOHelpers::GetVectorFromMortonCode( target_node.MortonCode ) );
    neighbor_coords += SVONavigationConstants::NeighborDirections[ direction ];

    if ( neighbor_coords.X < 0 || neighbor_coords.X >= max_coordinates ||
         neighbor_coords.Y < 0 || neighbor_coords.Y >= max_coordinates ||
         neighbor_coords.Z < 0 || neighbor_coords.Z >= max_coordinates )
    {
        node_address.Invalidate();
        return true;
    }

    const auto neighbor_code = FSVOHelpers::GetMortonCodeFromVector( neighbor_coords );

    int32 stop_index = layer_nodes_count;
    int32 increment = 1;

    if ( neighbor_code < target_node.MortonCode )
    {
        increment = -1;
        stop_index = -1;
    }

    for ( int32 neighbor_node_index = node_index + increment; neighbor_node_index != stop_index; neighbor_node_index += increment )
    {
        auto & node = layer_nodes[ neighbor_node_index ];

        if ( node.MortonCode == neighbor_code )
        {
            if ( layer_index == 0 &&
                 node.HasChildren() &&
                 SVOData.GetLeafNodes().GetLeafNode( node.FirstChild.NodeIndex ).IsCompletelyOccluded() )
            {
                node_address.Invalidate();
                return true;
            }

            node_address.LayerIndex = layer_index;

            if ( neighbor_node_index >= layer_nodes_count || neighbor_node_index < 0 )
            {
                break;
            }

            node_address.NodeIndex = neighbor_node_index;

            return true;
        }

        // If we've passed the code we're looking for, it's not on this layer
        if ( increment == -1 && node.MortonCode < neighbor_code || increment == 1 && node.MortonCode > neighbor_code )
        {
            return false;
        }
    }
    return false;
}

void FSVOVolumeNavigationData::BuildParentLinkForLeafNodes( const TMap<LeafIndex, MortonCode> & leaf_index_to_parent_morton_code_map )
{
    for ( const auto & key_pair : leaf_index_to_parent_morton_code_map )
    {
        auto & leaf_node = SVOData.GetLeafNodes().GetLeafNode( key_pair.Key );
        leaf_node.Parent.LayerIndex = 1;

        const auto node_index = GetNodeIndexFromMortonCode( 1, key_pair.Value );
        check( node_index != INDEX_NONE );

        leaf_node.Parent.NodeIndex = node_index;
    }
}

void FSVOVolumeNavigationData::GetNodeNeighbors( TArray< FSVONodeAddress > & neighbors, const FSVONodeAddress & node_address ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetNeighbors );

    const auto & node = GetNodeFromAddress( node_address );
    if ( node_address.LayerIndex == 0 && node.FirstChild.IsValid() )
    {
        GetLeafNeighbors( neighbors, node_address );
        return;
    }

    for ( NeighborDirection neighbor_direction = 0; neighbor_direction < 6; neighbor_direction++ )
    {
        const auto & neighbor_address = node.Neighbors[ neighbor_direction ];

        if ( !neighbor_address.IsValid() )
        {
            continue;
        }

        const auto & neighbor = GetNodeFromAddress( neighbor_address );

        if ( !neighbor.HasChildren() )
        {
            neighbors.Add( neighbor_address );
            continue;
        }

        TArray< FSVONodeAddress > neighbor_addresses_working_set;
        neighbor_addresses_working_set.Push( neighbor_address );

        while ( neighbor_addresses_working_set.Num() > 0 )
        {
            // Pop off the top of the working set
            auto this_address = neighbor_addresses_working_set.Pop();

            const auto & this_node = GetNodeFromAddress( this_address );

            // If the node as no children, it's clear, so add to neighbors and continue
            if ( !this_node.HasChildren() )
            {
                neighbors.Add( neighbor_address );
                continue;
            }

            if ( this_address.LayerIndex > 0 )
            {
                /* Morton code node ordering
                    Z
                    ^
                    |          5 --- 7
                    |        / |   / |
                    |       4 --- 6  |
                    |  X    |  1 -|- 3
                    | /     | /   | /
                    |/      0 --- 2
                    +-------------------> Y
                */

                static constexpr NodeIndex ChildOffsetsDirections[ 6 ][ 4 ] = {
                    { 0, 4, 2, 6 },
                    { 1, 3, 5, 7 },
                    { 0, 1, 4, 5 },
                    { 2, 3, 6, 7 },
                    { 0, 1, 2, 3 },
                    { 4, 5, 6, 7 }
                };

                // If it's above layer 0, we will need to potentially add 4 children using our offsets
                for ( const auto & child_index : ChildOffsetsDirections[ neighbor_direction ] )
                {
                    auto first_child_address = this_node.FirstChild;
                    first_child_address.NodeIndex += child_index;
                    const auto & child_node = GetNodeFromAddress( first_child_address );

                    if ( child_node.HasChildren() ) // If it has children, add them to the working set to keep going down
                    {
                        neighbor_addresses_working_set.Emplace( first_child_address );
                    }
                    else
                    {
                        neighbors.Emplace( first_child_address );
                    }
                }
            }
            else
            {
                /*
                Sub node morton code ordering for the face pointing to neighbor[0], which is (1,0,0)
                Use the debug draw options of the navigation data in the scene to show all the sub nodes
                 
                Z
                |
                |   36 38 52 54
                |   32 34 48 50
                |   04 06 20 22
                |   00 02 16 18
                |
                ------------------ Y
                */

                static constexpr NodeIndex LeafChildOffsetsDirections[ 6 ][ 16 ] = {
                    { 0, 2, 16, 18, 4, 6, 20, 22, 32, 34, 48, 50, 36, 38, 52, 54 },
                    { 9, 11, 25, 27, 13, 15, 29, 31, 41, 43, 57, 59, 45, 47, 61, 63 },
                    { 0, 1, 8, 9, 4, 5, 12, 13, 32, 33, 40, 41, 36, 37, 44, 45 },
                    { 18, 19, 26, 27, 22, 23, 30, 31, 50, 51, 58, 59, 54, 55, 62, 63 },
                    { 0, 1, 8, 9, 2, 3, 10, 11, 16, 17, 24, 25, 18, 19, 26, 27 },
                    { 36, 37, 44, 45, 38, 39, 46, 47, 52, 53, 60, 61, 54, 55, 62, 63 }
                };

                // If this is a leaf layer, then we need to add whichever of the 16 facing leaf nodes aren't blocked
                for ( const auto & leaf_index : LeafChildOffsetsDirections[ neighbor_direction ] )
                {
                    // Each of the childnodes
                    auto first_child_address = neighbor.FirstChild;
                    const auto & leaf_node = SVOData.GetLeafNodes().GetLeafNode( first_child_address.NodeIndex );

                    first_child_address.LayerIndex = 0;
                    first_child_address.NodeIndex = this_address.NodeIndex;
                    first_child_address.SubNodeIndex = leaf_index;

                    if ( !leaf_node.IsSubNodeOccluded( leaf_index ) )
                    {
                        neighbors.Emplace( first_child_address );
                    }
                }
            }
        }
    }
}

void FSVOVolumeNavigationData::GetLeafNeighbors( TArray< FSVONodeAddress > & neighbors, const FSVONodeAddress & leaf_address ) const
{
    QUICK_SCOPE_CYCLE_COUNTER( STAT_SVOBoundsNavigationData_GetLeafNeighbors );

    const MortonCode leaf_index = leaf_address.SubNodeIndex;
    const FSVONode & node = GetNodeFromAddress( leaf_address );
    const FSVOLeafNode & leaf = SVOData.GetLeafNodes().GetLeafNode( node.FirstChild.NodeIndex );

    uint_fast32_t x = 0, y = 0, z = 0;
    morton3D_64_decode( leaf_index, x, y, z );

    for ( NeighborDirection neighbor_direction = 0; neighbor_direction < 6; neighbor_direction++ )
    {
        FIntVector neighbor_coords( x, y, z );
        neighbor_coords += SVONavigationConstants::NeighborDirections[ neighbor_direction ];

        // If the neighbor is in bounds of this leaf node
        if ( neighbor_coords.X >= 0 && neighbor_coords.X < 4 && neighbor_coords.Y >= 0 && neighbor_coords.Y < 4 && neighbor_coords.Z >= 0 && neighbor_coords.Z < 4 )
        {
            const MortonCode sub_node_index = FSVOHelpers::GetMortonCodeFromVector( neighbor_coords );
            // If this node is not blocked, this is a valid address, add it
            if ( !leaf.IsSubNodeOccluded( sub_node_index ) )
            {
                neighbors.Emplace( FSVONodeAddress( 0, leaf_address.NodeIndex, sub_node_index ) );
            }
        }
        else // the neighbor is out of bounds, we need to find our neighbor
        {
            const FSVONodeAddress & neighbor_address = node.Neighbors[ neighbor_direction ];
            const FSVONode & neighbor_node = GetNodeFromAddress( neighbor_address );

            // If the neighbor layer 0 has no leaf nodes, just return it
            if ( !neighbor_node.FirstChild.IsValid() )
            {
                neighbors.Add( neighbor_address );
                continue;
            }

            const FSVOLeafNode & leaf_node = SVOData.GetLeafNodes().GetLeafNode( neighbor_node.FirstChild.NodeIndex );

            // leaf not occluded. Find the correct subnode
            if ( !leaf_node.IsCompletelyOccluded() )
            {
                if ( neighbor_coords.X < 0 )
                {
                    neighbor_coords.X = 3;
                }
                else if ( neighbor_coords.X > 3 )
                {
                    neighbor_coords.X = 0;
                }
                else if ( neighbor_coords.Y < 0 )
                {
                    neighbor_coords.Y = 3;
                }
                else if ( neighbor_coords.Y > 3 )
                {
                    neighbor_coords.Y = 0;
                }
                else if ( neighbor_coords.Z < 0 )
                {
                    neighbor_coords.Z = 3;
                }
                else if ( neighbor_coords.Z > 3 )
                {
                    neighbor_coords.Z = 0;
                }

                const MortonCode sub_node_index = FSVOHelpers::GetMortonCodeFromVector( neighbor_coords );

                // Only return the neighbor if it isn't blocked!
                if ( !leaf_node.IsSubNodeOccluded( sub_node_index ) )
                {
                    neighbors.Emplace( FSVONodeAddress( 0, neighbor_node.FirstChild.NodeIndex, sub_node_index ) );
                }
            }
            // else the leaf node is completely blocked, we don't return it
        }
    }
}
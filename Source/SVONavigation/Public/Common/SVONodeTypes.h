#pragma once

#include <CoreMinimal.h>
#include "SVONodeTypes.generated.h"

typedef uint_fast64_t MortonCode;
typedef uint8 LayerIndex;
typedef uint32 NodeIndex;
typedef int32 LeafIndex;
typedef uint8 SubNodeIndex;
typedef uint8 NeighborDirection;

USTRUCT()
struct FSVONodeAddress
{
    GENERATED_BODY()

    FSVONodeAddress() :
        LayerIndex( 15 ),
        NodeIndex( 0 ),
        SubNodeIndex( 0 )
    {
    }

    explicit FSVONodeAddress( const NavNodeRef NavRef )
    {
        const uint32 Ref32 = static_cast<uint32>(NavRef);
        LayerIndex = (Ref32 >> 28) & 0xF;
        NodeIndex = (Ref32 >> 6) & 0x3FFFFF;
        SubNodeIndex = Ref32 & 0x3F;
    }

    FSVONodeAddress( const LayerIndex layer_index, const MortonCode node_index, const SubNodeIndex sub_node_index = 0 ) :
        LayerIndex( layer_index ),
        NodeIndex( node_index ),
        SubNodeIndex( sub_node_index )
    {
    }

    bool IsValid() const;
    void Invalidate();

    bool operator==( const FSVONodeAddress & other ) const
    {
        return LayerIndex == other.LayerIndex && NodeIndex == other.NodeIndex && SubNodeIndex == other.SubNodeIndex;
    }

    bool operator!=( const FSVONodeAddress & other ) const
    {
        return !operator==( other );
    }

    NavNodeRef GetNavNodeRef() const
    {
        const int32 address = LayerIndex << 28 | NodeIndex << 6 | SubNodeIndex;
        return static_cast< NavNodeRef >( address );
    }

    FString ToString() const
    {
        return FString::Printf( TEXT( "%i %i %i" ), LayerIndex, NodeIndex, SubNodeIndex );
    }

    static const FSVONodeAddress InvalidAddress;

    uint8 LayerIndex        : 4;
    uint32 NodeIndex        : 22;
    uint8 SubNodeIndex      : 6;
};

FORCEINLINE bool FSVONodeAddress::IsValid() const
{
    return LayerIndex != 15;
}

FORCEINLINE void FSVONodeAddress::Invalidate()
{
    LayerIndex = 15;
}

FORCEINLINE uint32 GetTypeHash( const FSVONodeAddress & address )
{
    return HashCombine( HashCombine( GetTypeHash( address.LayerIndex ), GetTypeHash( address.NodeIndex ) ), GetTypeHash( address.SubNodeIndex ) );
}

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVONodeAddress & data )
{
    archive.Serialize( &data, sizeof( FSVONodeAddress ) );
    return archive;
}

struct FSVOLeafNode
{
    void MarkSubNodeAsOccluded( const SubNodeIndex index );
    bool IsSubNodeOccluded( const MortonCode morton_code ) const;
    bool IsCompletelyOccluded() const;
    bool IsCompletelyFree() const;

    uint_fast64_t SubNodes = 0;
    FSVONodeAddress Parent;
};

FORCEINLINE void FSVOLeafNode::MarkSubNodeAsOccluded( const SubNodeIndex index )
{
    SubNodes |= 1ULL << index;
}

FORCEINLINE bool FSVOLeafNode::IsSubNodeOccluded( const MortonCode morton_code ) const
{
    return ( SubNodes & 1ULL << morton_code ) != 0;
}

FORCEINLINE bool FSVOLeafNode::IsCompletelyOccluded() const
{
    return SubNodes == -1;
}

FORCEINLINE bool FSVOLeafNode::IsCompletelyFree() const
{
    return SubNodes == 0;
}

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVOLeafNode & data )
{
    archive << data.SubNodes;
    archive << data.Parent;
    return archive;
}

struct FSVONode
{
    FSVONode();
    explicit FSVONode( MortonCode morton_code );
    bool HasChildren() const;

    MortonCode MortonCode;
    FSVONodeAddress Parent;
    FSVONodeAddress FirstChild;
    FSVONodeAddress Neighbors[ 6 ];
};

FORCEINLINE bool FSVONode::HasChildren() const
{
    return FirstChild.IsValid();
}

FORCEINLINE bool operator<( const FSVONode & left, const FSVONode & right )
{
    return left.MortonCode < right.MortonCode;
}

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVONode & data )
{
    archive << data.MortonCode;
    archive << data.Parent;
    archive << data.FirstChild;

    for ( int32 neighbor_index = 0; neighbor_index < 6; neighbor_index++ )
    {
        archive << data.Neighbors[ neighbor_index ];
    }

    return archive;
}
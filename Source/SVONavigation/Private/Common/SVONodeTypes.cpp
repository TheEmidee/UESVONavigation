#include "Common/SVONodeTypes.h"

const FSVONodeAddress FSVONodeAddress::InvalidAddress;

FSVONode::FSVONode() :
	MortonCode( 0 ),
	Parent( FSVONodeAddress::InvalidAddress ),
	FirstChild( FSVONodeAddress::InvalidAddress )
{
}

FSVONode::FSVONode( const ::MortonCode morton_code ) :
	MortonCode( morton_code ),
	Parent( FSVONodeAddress::InvalidAddress ),
	FirstChild( FSVONodeAddress::InvalidAddress )
{
}
#pragma once

#include "CoreMinimal.h"

namespace SVONavigationConstants
{
	static const FIntVector NeighborDirections[ 6 ] = {
		{ 1, 0, 0 },
		{ -1, 0, 0 },
		{ 0, 1, 0 },
		{ 0, -1, 0 },
		{ 0, 0, 1 },
		{ 0, 0, -1 }
	};
}
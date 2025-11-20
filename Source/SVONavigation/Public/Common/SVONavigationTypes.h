#pragma once

#include <CoreMinimal.h>
#include "SVONodeTypes.h"
#include "SVODataTypes.h"

#include "SVONavigationTypes.generated.h"

class USVOPathFindingAlgorithm;
class USVOPathHeuristicCalculator;
class USVOPathTraversalCostCalculator;

DECLARE_DELEGATE_ThreeParams( FSVONavigationPathQueryDelegate, uint32, ENavigationQueryResult::Type, FNavPathSharedPtr );

USTRUCT()
struct FSVODataGenerationSettings
{
	GENERATED_USTRUCT_BODY()

	FSVODataGenerationSettings()
	{
		CollisionChannel = ECollisionChannel::ECC_WorldStatic;
		Clearance = 0.0f;

		CollisionQueryParameters.bFindInitialOverlaps = true;
		CollisionQueryParameters.bTraceComplex = false;
		CollisionQueryParameters.TraceTag = "SVONavigationRasterize";
	}

	UPROPERTY( EditAnywhere, Category = "Generation" )
	TEnumAsByte< ECollisionChannel > CollisionChannel;

	UPROPERTY( EditAnywhere, Category = "Generation" )
	float Clearance;

	FCollisionQueryParams CollisionQueryParameters;
};
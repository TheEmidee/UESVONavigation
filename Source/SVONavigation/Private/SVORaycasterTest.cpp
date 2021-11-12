#include "SVORaycasterTest.h"

#include "SVONavigationData.h"
#include "SVORaycaster.h"

#include <Components/SphereComponent.h>

ASVORaycasterTest::ASVORaycasterTest()
{
    SphereComponent = CreateDefaultSubobject< USphereComponent >( TEXT( "SphereComponent" ) );
    SphereComponent->InitSphereRadius( 100.0f );
    RootComponent = SphereComponent;

    PrimaryActorTick.bCanEverTick = false;

    Raycaster = NewObject< USVORayCaster_OctreeTraversal >();
    NavAgentProperties.PreferredNavData = ASVONavigationData::StaticClass();
    NavAgentProperties.AgentRadius = 100.0f;
}

void ASVORaycasterTest::DoRaycast()
{
    if ( OtherActor == nullptr )
    {
        return;
    }

    if ( Raycaster == nullptr )
    {
        return;
    }

    const auto has_los = Raycaster->HasLineOfSight( this, GetActorLocation(), OtherActor->GetActorLocation(), NavAgentProperties );
}

#include "SVORaycasterTest.h"

#include "SVORaycaster.h"

ASVORaycasterTest::ASVORaycasterTest()
{
    PrimaryActorTick.bCanEverTick = false;

    Raycaster = NewObject< USVORayCaster_OctreeTraversal >();
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

    Raycaster->HasLineOfSight( this, , )
}

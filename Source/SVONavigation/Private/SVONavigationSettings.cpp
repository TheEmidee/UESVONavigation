#include "SVONavigationSettings.h"

#include "Raycasters/SVORaycaster_OctreeTraversal.h"

USVONavigationSettings::USVONavigationSettings()
{
    bNavigationAutoUpdateEnabled = true;
    DefaultLineOfSightClass = USVORayCaster_OctreeTraversal::StaticClass();
}

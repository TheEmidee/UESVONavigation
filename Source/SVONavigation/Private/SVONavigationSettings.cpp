#include "SVONavigationSettings.h"

#include "Raycasters/SVORaycaster_OctreeTraversal.h"

USVONavigationSettings::USVONavigationSettings()
{
    bNavigationAutoUpdateEnabled = true;
    DefaultRaycasterClass = USVORayCaster_OctreeTraversal::StaticClass();
}

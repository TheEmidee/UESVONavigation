#include "SVONavigationSettings.h"

#include "Raycasters/SVORaycaster_OctreeTraversal.h"

USVONavigationSettings::USVONavigationSettings()
{
    ShouldDiscardSubLevelNavigationData = true;
    NavigationAutoUpdateEnabled = true;
    DefaultLineOfSightClass = USVORayCaster_OctreeTraversal::StaticClass();
}

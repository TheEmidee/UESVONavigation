#include "SVONavigationDataChunk.h"

void USVONavigationDataChunk::ReleaseNavigationData()
{
    NavigationData.Reset();
}

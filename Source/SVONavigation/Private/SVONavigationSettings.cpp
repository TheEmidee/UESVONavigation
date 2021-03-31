#include "SVONavigationSettings.h"

#if WITH_EDITOR
void USVONavigationSettings::PostEditChangeProperty( FPropertyChangedEvent & property_changed_event )
{
    Super::PostEditChangeProperty( property_changed_event );
    OnPropertyChangedDelegate.Broadcast();
}
#endif
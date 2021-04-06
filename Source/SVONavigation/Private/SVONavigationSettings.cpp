#include "SVONavigationSettings.h"

#include "SVONavigationSystem.h"

#if WITH_EDITOR
void USVONavigationSettings::PostEditChangeProperty( FPropertyChangedEvent & property_changed_event )
{
    Super::PostEditChangeProperty( property_changed_event );
    OnPropertyChangedDelegate.Broadcast();

    if ( auto * navigation_system = GEngine->GetEngineSubsystem< USVONavigationSystem >() )
    {
        navigation_system->UpdateAllNavigationVolumes();
    }
}
#endif
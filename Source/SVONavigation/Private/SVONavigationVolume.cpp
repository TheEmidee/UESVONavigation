#include "SVONavigationVolume.h"

#include "SVONavigationSystem.h"

#if WITH_EDITOR
void ASVONavigationVolume::PostEditUndo()
{
    Super::PostEditUndo();

    if ( !GIsEditor )
    {
        return;
    }
    if ( auto * navigation_system = GEngine->GetEngineSubsystem< USVONavigationSystem >() )
    {
        navigation_system->OnNavigationVolumeUpdated( *this );
    }
}

void ASVONavigationVolume::PostEditChangeProperty( FPropertyChangedEvent & PropertyChangedEvent )
{
    Super::PostEditChangeProperty( PropertyChangedEvent );

    if ( !GIsEditor )
    {
        return;
    }

    if ( auto * navigation_system = GEngine->GetEngineSubsystem< USVONavigationSystem >() )
    {
        const FName property_name = ( PropertyChangedEvent.Property != nullptr ) ? PropertyChangedEvent.Property->GetFName() : FName();
        const FName member_name = ( PropertyChangedEvent.MemberProperty != nullptr ) ? PropertyChangedEvent.MemberProperty->GetFName() : FName();

        if ( property_name == GET_MEMBER_NAME_CHECKED( ABrush, BrushBuilder ) 
            || member_name == USceneComponent::GetRelativeLocationPropertyName() 
            || member_name == USceneComponent::GetRelativeRotationPropertyName() 
            || member_name == USceneComponent::GetRelativeScale3DPropertyName() 
            )
        {
            navigation_system->OnNavigationVolumeUpdated( *this );
        }
    }
}

#endif

void ASVONavigationVolume::PostRegisterAllComponents()
{
    Super::PostRegisterAllComponents();

    if ( GetLocalRole() != ROLE_Authority )
    {
        return;
    }

    if ( auto * navigation_system = GEngine->GetEngineSubsystem< USVONavigationSystem >() )
    {
        navigation_system->OnNavigationVolumeAdded( *this );
    }
}

void ASVONavigationVolume::PostUnregisterAllComponents()
{
    Super::PostUnregisterAllComponents();

    if ( GetLocalRole() != ROLE_Authority )
    {
        return;
    }

    if ( auto * navigation_system = GEngine->GetEngineSubsystem< USVONavigationSystem >() )
    {
        navigation_system->OnNavigationVolumeRemoved( *this );
    }
}
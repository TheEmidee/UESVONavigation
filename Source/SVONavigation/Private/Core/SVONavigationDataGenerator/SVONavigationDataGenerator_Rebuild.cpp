#include "SVONavigationDataGenerator.h"
#include "SVONavigationData.h"
#include "NavigationSystem.h"
#include "GameFramework/PlayerController.h"

bool FSVONavigationDataGenerator::RebuildAll()
{
    NavigationData.UpdateNavVersion();

    UpdateNavigationBounds();

    TArray< FNavigationDirtyArea > dirty_areas;
    dirty_areas.Reserve( RegisteredNavigationBounds.Num() );

    for ( const auto & registered_navigation_bounds : RegisteredNavigationBounds )
    {
        dirty_areas.Emplace( FNavigationDirtyArea( registered_navigation_bounds, ENavigationDirtyFlag::All ) );
    }

    RebuildDirtyAreas( dirty_areas );

    NavigationData.RequestDrawingUpdate();
    return true;
}

void FSVONavigationDataGenerator::OnNavigationBoundsChanged()
{
    UpdateNavigationBounds();
}

void FSVONavigationDataGenerator::RebuildDirtyAreas( const TArray< FNavigationDirtyArea > & dirty_areas )
{
    // The dirty areas are not always in the navigation bounds. If we move a static mesh outside of the navigation bounds, that function is called nonetheless
    // So let's first keep only the areas which are in the known navigation bounds
    for ( const auto & dirty_area : dirty_areas )
    {
        const auto matching_bounds = RegisteredNavigationBounds.FilterByPredicate( [ &dirty_area ]( const FBox & box ) {
            return box == dirty_area.Bounds || box.IsInside( dirty_area.Bounds ) || box.Intersect( dirty_area.Bounds );
        } );

        for ( const auto & matching_bounds_element : matching_bounds )
        {
            // Don't add another pending generation if one is already there for the navigation bounds the dirty area is in
            if ( PendingBoundsDataGenerationElements.FindByPredicate( [ &matching_bounds_element ]( const FPendingBoundsDataGenerationElement & pending_element ) {
                     return pending_element.VolumeBounds == matching_bounds_element;
                 } ) == nullptr )
            {
                FPendingBoundsDataGenerationElement pending_box_element;
                pending_box_element.VolumeBounds = matching_bounds_element;
                PendingBoundsDataGenerationElements.Emplace( pending_box_element );

                NavigationData.RemoveDataInBounds( matching_bounds_element );
            }
        }
    }

    // Sort tiles by proximity to players
    if ( PendingBoundsDataGenerationElements.Num() > 0 )
    {
        SortPendingBounds();
    }
}

bool FSVONavigationDataGenerator::IsBuildInProgressCheckDirty() const
{
    return RunningBoundsDataGenerationElements.Num() || PendingBoundsDataGenerationElements.Num();
}

int32 FSVONavigationDataGenerator::GetNumRemaningBuildTasks() const
{
    return RunningBoundsDataGenerationElements.Num() + PendingBoundsDataGenerationElements.Num();
}

int32 FSVONavigationDataGenerator::GetNumRunningBuildTasks() const
{
    return RunningBoundsDataGenerationElements.Num();
}

void FSVONavigationDataGenerator::GetSeedLocations( TArray< FVector2D > & seed_locations, UWorld & world ) const
{
    // Collect players positions
    for ( FConstPlayerControllerIterator player_iterator = world.GetPlayerControllerIterator(); player_iterator; ++player_iterator )
    {
        if ( const auto * player_controller = player_iterator->Get() )
        {
            if ( const APawn * pawn = player_controller->GetPawn() )
            {
                const FVector2D seed_location( pawn->GetActorLocation() );
                seed_locations.Add( seed_location );
            }
        }
    }
}

void FSVONavigationDataGenerator::SortPendingBounds()
{
    if ( UWorld * current_world = GetWorld() )
    {
        TArray< FVector2D > seed_locations;
        GetSeedLocations( seed_locations, *current_world );

        if ( seed_locations.Num() == 0 )
        {
            seed_locations.Add( FVector2D( TotalNavigationBounds.GetCenter() ) );
        }

        if ( seed_locations.Num() > 0 )
        {
            for ( auto & element : PendingBoundsDataGenerationElements )
            {
                FVector2D tile_center_2d = FVector2D( element.VolumeBounds.GetCenter() );
                for ( const auto & seed_location : seed_locations )
                {
                    element.SeedDistance = FMath::Min( element.SeedDistance, FVector2D::DistSquared( tile_center_2d, seed_location ) );
                }
            }

            PendingBoundsDataGenerationElements.Sort();
        }
    }
}

void FSVONavigationDataGenerator::UpdateNavigationBounds()
{
    if ( const UNavigationSystemV1 * navigation_system = FNavigationSystem::GetCurrent< UNavigationSystemV1 >( GetWorld() ) )
    {
        if ( !navigation_system->ShouldGenerateNavigationEverywhere() )
        {
            FBox bounds_sum( ForceInit );
            {
                TArray< FBox > supported_navigation_bounds;
                navigation_system->GetNavigationBoundsForNavData( NavigationData, supported_navigation_bounds );

                RegisteredNavigationBounds.Reset( supported_navigation_bounds.Num() );

                for ( const auto & box : supported_navigation_bounds )
                {
                    RegisteredNavigationBounds.Add( box );
                    bounds_sum += box;
                }

                // :NOTE: Commented because starting in UE5 or UE5.1 it will always remove all nav data
                // Can be removed later when it's sure this can be dropped
                // Remove the existing navigation bounds which don't match the new navigation bounds
                // NavigationData.RemoveDataInBounds( RegisteredNavigationBounds );
            }
            TotalNavigationBounds = bounds_sum;
        }
        else
        {
            RegisteredNavigationBounds.Reset( 1 );
            TotalNavigationBounds = navigation_system->GetWorldBounds();
            if ( !TotalNavigationBounds.IsValid )
            {
                RegisteredNavigationBounds.Add( TotalNavigationBounds );
            }
        }
    }
    else
    {
        TotalNavigationBounds = FBox( ForceInit );
    }
}

void FSVONavigationDataGenerator::RebuildBounds(const TArray<FBox>& BoundsToRebuild)
{
    NavigationData.UpdateNavVersion();
    
    for (const FBox& BuildBounds : BoundsToRebuild)
    {
        // Don't add another pending generation if one is already there for these bounds.
        if (PendingBoundsDataGenerationElements.FindByPredicate([&BuildBounds](const FPendingBoundsDataGenerationElement& PendingElement) {
            return PendingElement.VolumeBounds == BuildBounds;
        }) == nullptr)
        {
            FPendingBoundsDataGenerationElement PendingBoxElement;
            PendingBoxElement.VolumeBounds = BuildBounds;
            PendingBoundsDataGenerationElements.Emplace(PendingBoxElement);

            NavigationData.RemoveDataInBounds(BuildBounds);
        }
    }
    
    if (PendingBoundsDataGenerationElements.Num() > 0)
    {
        SortPendingBounds();
    }
}
#include "SVONavigationData.h"

#include "SVONavDataRenderingComponent.h"
#include "SVONavigationSettings.h"
#include "DrawDebugHelpers.h"
#include "NavigationSystem.h"

#if WITH_EDITOR
#include "ObjectEditorUtils.h"
#endif

UPrimitiveComponent * ASVONavigationData::ConstructRenderingComponent()
{
    return NewObject< USVONavDataRenderingComponent >( this, TEXT( "SVONavRenderingComp" ), RF_Transient );
}

void ASVONavigationData::TickActor( const float delta_time, const ELevelTick tick, FActorTickFunction & this_tick_function )
{
    Super::TickActor( delta_time, tick, this_tick_function );

#if ENABLE_DRAW_DEBUG

    if ( bEnableDrawing && DebugInfos.bDebugDrawActivePaths )
    {
        for ( auto active_path : ActivePaths )
        {
            if ( !active_path.IsValid() )
            {
                continue;
            }

            const TSharedPtr< FNavigationPath, ESPMode::ThreadSafe > active_path_ptr = active_path.Pin();
            const auto & path_points = active_path_ptr->GetPathPoints();

            for ( auto path_point_index = 1; path_point_index < path_points.Num(); ++path_point_index )
            {
                const auto & from = path_points[ path_point_index - 1 ].Location;
                const auto & to = path_points[ path_point_index ].Location;

                DrawDebugLine( GetWorld(), from, to, FColor::Red, false, -1, SDPG_World, 5.0f );
                DrawDebugCone( GetWorld(), to, from - to, 50.0f, 0.25f, 0.25f, 16, FColor::Red, false, -1, SDPG_World, 5.0f );
            }
        }
    }

#endif
}

#if !UE_BUILD_SHIPPING
uint32 ASVONavigationData::LogMemUsed() const
{
    const auto super_mem_used = Super::LogMemUsed();

    auto navigation_mem_size = 0;
    for ( const auto & nav_bounds_data : VolumeNavigationData )
    {
        const auto octree_data_mem_size = nav_bounds_data.GetData().GetAllocatedSize();
        navigation_mem_size += octree_data_mem_size;
    }
    const auto mem_used = super_mem_used + navigation_mem_size;

    UE_LOG( LogNavigation, Warning, TEXT( "%s: ASVONavigationData: %u\n    self: %d" ), *GetName(), mem_used, sizeof( ASVONavigationData ) );

    return mem_used;
}
#endif

void ASVONavigationData::RequestDrawingUpdate( const bool force )
{
#if !UE_BUILD_SHIPPING
    if ( force || USVONavDataRenderingComponent::IsNavigationShowFlagSet( GetWorld() ) )
    {
        if ( force )
        {
            if ( USVONavDataRenderingComponent * rendering_component = Cast< USVONavDataRenderingComponent >( RenderingComp ) )
            {
                rendering_component->ForceUpdate();
            }
        }

        DECLARE_CYCLE_STAT( TEXT( "FSimpleDelegateGraphTask.Requesting SVO navmesh redraw" ),
            STAT_FSimpleDelegateGraphTask_RequestingNavmeshRedraw,
            STATGROUP_TaskGraphTasks );

        FSimpleDelegateGraphTask::CreateAndDispatchWhenReady(
            FSimpleDelegateGraphTask::FDelegate::CreateUObject( this, &ASVONavigationData::UpdateDrawing ),
            GET_STATID( STAT_FSimpleDelegateGraphTask_RequestingNavmeshRedraw ),
            nullptr,
            ENamedThreads::GameThread );
    }
#endif // !UE_BUILD_SHIPPING
}

void ASVONavigationData::UpdateDrawing() const
{
#if !UE_BUILD_SHIPPING
    if ( USVONavDataRenderingComponent * rendering_component = Cast< USVONavDataRenderingComponent >( RenderingComp ) )
    {
        if ( rendering_component->GetVisibleFlag() && ( rendering_component->UpdateIsForced() || USVONavDataRenderingComponent::IsNavigationShowFlagSet( GetWorld() ) ) )
        {
            rendering_component->MarkRenderStateDirty();
        }
    }
#endif
}

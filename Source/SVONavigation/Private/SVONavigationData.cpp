#include "SVONavigationData.h"

#include "NavigationSystem.h"
#include "SVONavDataRenderingComponent.h"
#include "SVONavigationDataGenerator.h"
#include "SVONavigationPath.h"
#include "SVONavigationSettings.h"

#include <AI/NavDataGenerator.h>

ASVONavigationData::ASVONavigationData()
{
    PrimaryActorTick.bCanEverTick = false;
}

void ASVONavigationData::PostInitProperties()
{
    if ( UWorld * world = GetWorld() )
    {
        if ( auto * settings = GetDefault< USVONavigationSettings >() )
        {
            if ( HasAnyFlags( RF_NeedLoad )                                                                   //  was loaded
                 && settings->ShouldDiscardSubLevelNavigationData && GEngine->IsSettingUpPlayWorld() == false // this is a @HACK
                 && world->GetOutermost() != GetOutermost()
                 // If we are cooking, then let them all pass.
                 // They will be handled at load-time when running.
                 && !IsRunningCommandlet() )
            {
                // marking self for deletion
                CleanUpAndMarkPendingKill();
            }
        }
    }

    Super::PostInitProperties();
}

void ASVONavigationData::Serialize( FArchive & archive )
{
    Super::Serialize( archive );
    archive << NavigationBoundsData;
    archive << DebugInfos;
}

void ASVONavigationData::CleanUp()
{
    Super::CleanUp();
    ResetGenerator();
}

FBox ASVONavigationData::GetBounds() const
{
    FBox result( EForceInit::ForceInit );

    /*for ( const auto & key_pair : NavigationBoundsData )
    {
        result += key_pair.Value.GetBox();
    }*/

    return result;
}

FNavLocation ASVONavigationData::GetRandomPoint( FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    // :TODO:
    ensure( false );
    return FNavLocation();
}

bool ASVONavigationData::GetRandomReachablePointInRadius( const FVector & origin, float radius, FNavLocation & out_result, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    // :TODO:
    ensure( false );
    return false;
}

bool ASVONavigationData::GetRandomPointInNavigableRadius( const FVector & origin, float Radius, FNavLocation & out_result, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    // :TODO:
    ensure( false );
    return false;
}

void ASVONavigationData::BatchRaycast( TArray< FNavigationRaycastWork > & workload, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    // :TODO:
    ensure( false );
}

bool ASVONavigationData::FindMoveAlongSurface( const FNavLocation & start_location, const FVector & target_position, FNavLocation & out_location, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    // :TODO:
    ensure( false );
    return false;
}

bool ASVONavigationData::ProjectPoint( const FVector & point, FNavLocation & out_location, const FVector & extent, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    // :TODO:
    ensure( false );
    return false;
}

void ASVONavigationData::BatchProjectPoints( TArray< FNavigationProjectionWork > & Workload, const FVector & Extent, FSharedConstNavQueryFilter Filter, const UObject * Querier ) const
{
    // :TODO:
    ensure( false );
}

void ASVONavigationData::BatchProjectPoints( TArray< FNavigationProjectionWork > & Workload, FSharedConstNavQueryFilter Filter, const UObject * Querier ) const
{
    // :TODO:
    ensure( false );
}

ENavigationQueryResult::Type ASVONavigationData::CalcPathCost( const FVector & path_start, const FVector & path_end, float & out_path_cost, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    // :TODO:
    ensure( false );
    return ENavigationQueryResult::Error;
}

ENavigationQueryResult::Type ASVONavigationData::CalcPathLength( const FVector & path_start, const FVector & path_end, float & out_path_length, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    // :TODO:
    ensure( false );
    return ENavigationQueryResult::Error;
}

ENavigationQueryResult::Type ASVONavigationData::CalcPathLengthAndCost( const FVector & path_start, const FVector & path_end, float & out_path_length, float & out_path_cost, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    // :TODO:
    ensure( false );
    return ENavigationQueryResult::Error;
}

bool ASVONavigationData::DoesNodeContainLocation( NavNodeRef node_ref, const FVector & world_space_location ) const
{
    // :TODO:
    ensure( false );
    return false;
}

UPrimitiveComponent * ASVONavigationData::ConstructRenderingComponent()
{
    return NewObject< USVONavDataRenderingComponent >( this, TEXT( "SVONavRenderingComp" ), RF_Transient );
}

void ASVONavigationData::OnStreamingLevelAdded( ULevel * level, UWorld * world )
{
    // :TODO:
    ensure( false );
}

void ASVONavigationData::OnStreamingLevelRemoved( ULevel * level, UWorld * world )
{
    // :TODO:
    ensure( false );
}

void ASVONavigationData::OnNavAreaChanged()
{
    // :TODO:
    ensure( false );
}

void ASVONavigationData::OnNavAreaAdded( const UClass * nav_area_class, int32 agent_index )
{
    // :TODO:
    ensure( false );
}

int32 ASVONavigationData::GetNewAreaID( const UClass * nav_area_class ) const
{
    // :TODO:
    ensure( false );
    return -1;
}

int32 ASVONavigationData::GetMaxSupportedAreas() const
{
    return 32;
}

#if WITH_EDITOR
void ASVONavigationData::PostEditChangeProperty( FPropertyChangedEvent & property_changed_event )
{
    Super::PostEditChangeProperty( property_changed_event );

    if ( property_changed_event.Property == nullptr )
    {
        return;
    }

    const FName property_name = property_changed_event.Property->GetFName();

    if ( property_name == GET_MEMBER_NAME_CHECKED( ASVONavigationData, GenerationSettings ) )
    {
        if ( auto * settings = GetDefault< USVONavigationSettings >() )
        {
            if ( !HasAnyFlags( RF_ClassDefaultObject ) && settings->NavigationAutoUpdateEnabled )
            {
                RebuildAll();
            }
        }
    }
}

bool ASVONavigationData::ShouldExport()
{
    return false;
}
#endif

#if !UE_BUILD_SHIPPING
uint32 ASVONavigationData::LogMemUsed() const
{
    // :TODO:
    ensure( false );
    return Super::LogMemUsed();
}
#endif

FSVOPathFindingResult ASVONavigationData::FindPath( const FSVOPathFindingQuery & path_finding_query ) const
{
    const ASVONavigationData * self = path_finding_query.NavigationData.Get();

    if ( self == nullptr )
    {
        return FSVOPathFindingResult( ENavigationQueryResult::Error );
    }

    FSVOPathFindingResult result( ENavigationQueryResult::Error );

    FNavigationPath * navigation_path = path_finding_query.PathInstanceToFill.Get();
    FSVONavigationPath * svo_navigation_path = navigation_path ? navigation_path->CastPath< FSVONavigationPath >() : nullptr;

    if ( svo_navigation_path )
    {
        result.Path = path_finding_query.PathInstanceToFill;
        svo_navigation_path->ResetForRepath();
    }
    else
    {
        result.Path = self->CreatePathInstance< FSVONavigationPath >( path_finding_query );
        navigation_path = result.Path.Get();
        svo_navigation_path = navigation_path ? navigation_path->CastPath< FSVONavigationPath >() : nullptr;
    }

    const FNavigationQueryFilter * NavFilter = path_finding_query.QueryFilter.Get();

    if ( svo_navigation_path != nullptr && NavFilter != nullptr )
    {
        const FVector adjusted_end_location = NavFilter->GetAdjustedEndLocation( path_finding_query.EndLocation );
        if ( ( path_finding_query.StartLocation - adjusted_end_location ).IsNearlyZero() )
        {
            result.Path->GetPathPoints().Reset();
            result.Path->GetPathPoints().Add( FNavPathPoint( adjusted_end_location ) );
            result.Result = ENavigationQueryResult::Success;
        }
        else
        {
            /*result.Result = RecastNavMesh->RecastNavMeshImpl->FindPath( path_finding_query.StartLocation, adjusted_end_location, path_finding_query.CostLimit, *svo_navigation_path, *NavFilter, path_finding_query.Owner.Get() );

            const bool bPartialPath = result.IsPartial();
            if ( bPartialPath )
            {
                result.Result = path_finding_query.bAllowPartialPaths ? ENavigationQueryResult::Success : ENavigationQueryResult::Fail;
            }*/
        }
    }

    return result;
}

void ASVONavigationData::ConditionalConstructGenerator()
{
    ResetGenerator();

    UWorld * world = GetWorld();
    check( world );
    const bool requires_generator = SupportsRuntimeGeneration() || !world->IsGameWorld();

    if ( !requires_generator )
    {
        return;
    }

    if ( FSVONavigationDataGenerator * generator = new FSVONavigationDataGenerator( *this ) )
    {
        NavDataGenerator = MakeShareable( static_cast< FNavDataGenerator * >( generator ) );
        generator->Init();
    }
}

void ASVONavigationData::RequestDrawingUpdate( bool force )
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

void ASVONavigationData::UpdateDrawing()
{
#if !UE_BUILD_SHIPPING
    if ( USVONavDataRenderingComponent * rendering_component = Cast< USVONavDataRenderingComponent >( RenderingComp ) )
    {
        if ( rendering_component->GetVisibleFlag() && ( rendering_component->IsForcingUpdate() || USVONavDataRenderingComponent::IsNavigationShowFlagSet( GetWorld() ) ) )
        {
            rendering_component->MarkRenderStateDirty();
        }
    }
#endif /
}

void ASVONavigationData::ResetGenerator( const bool cancel_build )
{
    if ( NavDataGenerator.IsValid() )
    {
        if ( cancel_build )
        {
            NavDataGenerator->CancelBuild();
        }

        NavDataGenerator.Reset();
    }
}

void ASVONavigationData::OnNavigationDataUpdatedInBounds( const TArray< FBox > & updated_boxes )
{
    //InvalidateAffectedPaths(ChangedTiles);
}

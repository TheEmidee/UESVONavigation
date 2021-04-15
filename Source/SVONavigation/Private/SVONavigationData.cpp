#include "SVONavigationData.h"

#include "SVONavDataRenderingComponent.h"
#include "SVONavigationDataGenerator.h"
#include "SVONavigationPath.h"
#include "SVONavigationSettings.h"
#include "SVOPathFinder.h"
#include "NavMesh/NavMeshPath.h"



#include <AI/NavDataGenerator.h>
#include <NavigationSystem.h>
#if WITH_EDITOR
#include <ObjectEditorUtils.h>
#endif

ASVONavigationData::ASVONavigationData()
{
    PrimaryActorTick.bCanEverTick = false;
    MaxSimultaneousBoxGenerationJobsCount = 1024;

    if ( !HasAnyFlags( RF_ClassDefaultObject ) )
    {
        FindPathImplementation = FindPath;
        /*FindHierarchicalPathImplementation = FindPath;

        TestPathImplementation = TestPath;
        TestHierarchicalPathImplementation = TestHierarchicalPath;*/

        //RaycastImplementation = NavMeshRaycast;

        //RecastNavMeshImpl = new FPImplRecastNavMesh( this );

        //// add predefined areas up front
        //SupportedAreas.Add( FSupportedAreaData( UNavArea_Null::StaticClass(), RECAST_NULL_AREA ) );
        //SupportedAreas.Add( FSupportedAreaData( UNavArea_LowHeight::StaticClass(), RECAST_LOW_AREA ) );
        //SupportedAreas.Add( FSupportedAreaData( UNavArea_Default::StaticClass(), RECAST_DEFAULT_AREA ) );
    }
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
    Super::OnStreamingLevelAdded( level, world );
}

void ASVONavigationData::OnStreamingLevelRemoved( ULevel * level, UWorld * world )
{
    Super::OnStreamingLevelRemoved( level, world );
}

void ASVONavigationData::OnNavAreaChanged()
{
    Super::OnNavAreaChanged();
}

void ASVONavigationData::OnNavAreaAdded( const UClass * nav_area_class, int32 agent_index )
{
    Super::OnNavAreaAdded( nav_area_class, agent_index );
}

int32 ASVONavigationData::GetNewAreaID( const UClass * nav_area_class ) const
{
    return Super::GetNewAreaID( nav_area_class );
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

    if ( property_changed_event.Property != nullptr )
    {
        const FName category_name = FObjectEditorUtils::GetCategoryFName( property_changed_event.Property );
        static const FName NAME_Generation = FName( TEXT( "Generation" ) );

        if ( category_name == NAME_Generation )
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

FPathFindingResult ASVONavigationData::FindPath( const FNavAgentProperties & /*agent_properties*/, const FPathFindingQuery & path_finding_query )
{
    const auto * self = Cast< ASVONavigationData >( path_finding_query.NavData.Get() );

    if ( self == nullptr )
    {
        return ENavigationQueryResult::Error;
    }

    FPathFindingResult result( ENavigationQueryResult::Error );

    FNavigationPath * navigation_path = path_finding_query.PathInstanceToFill.Get();
    FNavMeshPath * nav_mesh_path = navigation_path ? navigation_path->CastPath< FNavMeshPath >() : nullptr;

    if ( nav_mesh_path != nullptr )
    {
        result.Path = path_finding_query.PathInstanceToFill;
        nav_mesh_path->ResetForRepath();
    }
    else
    {
        result.Path = self->CreatePathInstance< FSVONavigationPath >( path_finding_query );
        navigation_path = result.Path.Get();
        nav_mesh_path = navigation_path ? navigation_path->CastPath< FNavMeshPath >() : nullptr;
    }

    if ( navigation_path != nullptr )
    {
        if ( const FNavigationQueryFilter * navigation_filter = path_finding_query.QueryFilter.Get() )
        {
            const FVector adjusted_end_location = path_finding_query.EndLocation;// navigation_filter->GetAdjustedEndLocation( path_finding_query.EndLocation );
            if ( ( path_finding_query.StartLocation - adjusted_end_location ).IsNearlyZero() )
            {
                result.Path->GetPathPoints().Reset();
                result.Path->GetPathPoints().Add( FNavPathPoint( adjusted_end_location ) );
                result.Result = ENavigationQueryResult::Success;
            }
            else
            {
                FSVOPathFinder path_finder( *self );
                result.Result = path_finder.GetPath( *result.Path.Get(), path_finding_query.StartLocation, adjusted_end_location, *navigation_filter );

                /*result.Result = RecastNavMesh->RecastNavMeshImpl->FindPath( path_finding_query.StartLocation, adjusted_end_location, path_finding_query.CostLimit, *svo_navigation_path, *NavFilter, path_finding_query.Owner.Get() );

                const bool bPartialPath = result.IsPartial();
                if ( bPartialPath )
                {
                    result.Result = path_finding_query.bAllowPartialPaths ? ENavigationQueryResult::Success : ENavigationQueryResult::Fail;
                }*/
            }
        }
    }

    return result;
}

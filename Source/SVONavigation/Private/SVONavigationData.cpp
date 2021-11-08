#include "SVONavigationData.h"

#include "NavMesh/NavMeshPath.h"
#include "SVONavDataRenderingComponent.h"
#include "SVONavigationDataGenerator.h"
#include "SVONavigationPath.h"
#include "SVONavigationQueryFilterImpl.h"
#include "SVONavigationSettings.h"
#include "SVOPathFinder.h"
#include "SVOPathFindingAlgorithm.h"
#include "SVOVersion.h"

#include <AI/NavDataGenerator.h>
#include <NavigationSystem.h>

#if WITH_EDITOR
#include <ObjectEditorUtils.h>
#endif

ASVONavigationData::ASVONavigationData() :
    Version( ESVOVersion::Latest )
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

    SVODataPtr = MakeUnique< FSVOData >();
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

    if ( HasAnyFlags( RF_ClassDefaultObject | RF_NeedLoad ) == false )
    {
        RecreateDefaultFilter();
    }
}

void ASVONavigationData::PostLoad()
{
    Super::PostLoad();

    RecreateDefaultFilter();
}

void ASVONavigationData::Serialize( FArchive & archive )
{
    Super::Serialize( archive );

    archive << Version;

    // Same as in RecastNavMesh
    uint32 svo_size_bytes = 0;
    const auto svo_size_position = archive.Tell();

    archive << svo_size_bytes;

    if ( archive.IsLoading() )
    {
        if ( Version < ESVOVersion::MinCompatible )
        {
            // incompatible, just skip over this data.  nav mesh needs rebuilt.
            archive.Seek( svo_size_position + svo_size_bytes );
            return;
        }
    }

    SVODataPtr->Serialize( archive, Version );

    if ( archive.IsSaving() )
    {
        const int64 current_position = archive.Tell();

        svo_size_bytes = current_position - svo_size_position;

        archive.Seek( svo_size_position );
        archive << svo_size_bytes;
        archive.Seek( current_position );
    }
}

void ASVONavigationData::CleanUp()
{
    Super::CleanUp();
    ResetGenerator();
}

bool ASVONavigationData::NeedsRebuild() const
{
    const auto needs_rebuild = SVODataPtr->NeedsRebuild();

    if ( NavDataGenerator.IsValid() )
    {
        return needs_rebuild || NavDataGenerator->GetNumRemaningBuildTasks() > 0;
    }

    return needs_rebuild;
}

void ASVONavigationData::EnsureBuildCompletion()
{
    Super::EnsureBuildCompletion();

    // Doing this as a safety net solution due to UE-20646, which was basically a result of random
    // over-releasing of default filter's shared pointer (it seemed). We might have time to get
    // back to this time some time in next 3 years :D
    RecreateDefaultFilter();
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
        static const FName NAME_Query = FName( TEXT( "Query" ) );

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
        else if ( category_name == NAME_Query )
        {
            RecreateDefaultFilter();
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
    const auto super_mem_used = Super::LogMemUsed();

    const auto mem_used = super_mem_used + SVODataPtr->GetAllocatedSize();

    UE_LOG( LogNavigation, Warning, TEXT( "%s: ASVONavigationData: %u\n    self: %d" ), *GetName(), mem_used, sizeof( ASVONavigationData ) );

    return mem_used;
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

void ASVONavigationData::RecreateDefaultFilter()
{
    DefaultQueryFilter->SetFilterType< FSVONavigationQueryFilterImpl >();
}

void ASVONavigationData::UpdateDrawing()
{
#if !UE_BUILD_SHIPPING
    if ( USVONavDataRenderingComponent * rendering_component = Cast< USVONavDataRenderingComponent >( RenderingComp ) )
    {
        if ( rendering_component->GetVisibleFlag() && ( rendering_component->UpdateIsForced() || USVONavDataRenderingComponent::IsNavigationShowFlagSet( GetWorld() ) ) )
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

void ASVONavigationData::ClearNavigationData()
{
    SVODataPtr->ClearData();
    RequestDrawingUpdate();
}

void ASVONavigationData::BuildNavigationData()
{
    RebuildAll();
}

FPathFindingResult ASVONavigationData::FindPath( const FNavAgentProperties & agent_properties, const FPathFindingQuery & path_finding_query )
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
            const FVector adjusted_end_location = path_finding_query.EndLocation; // navigation_filter->GetAdjustedEndLocation( path_finding_query.EndLocation );
            if ( ( path_finding_query.StartLocation - adjusted_end_location ).IsNearlyZero() )
            {
                result.Path->GetPathPoints().Reset();
                result.Path->GetPathPoints().Add( FNavPathPoint( adjusted_end_location ) );
                result.Result = ENavigationQueryResult::Success;
            }
            else
            {
                result.Result = FSVOPathFinder::GetPath( *result.Path.Get(), agent_properties, *self, path_finding_query.StartLocation, adjusted_end_location, path_finding_query );
            }
        }
    }

    return result;
}

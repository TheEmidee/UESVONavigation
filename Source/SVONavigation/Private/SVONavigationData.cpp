#include "SVONavigationData.h"

#include "PathFinding/SVONavigationQueryFilterImpl.h"
#include "PathFinding/SVOPathFinder.h"
#include "PathFinding/SVOPathFindingAlgorithm.h"
#include "SVOBoundsVolume.h"
#include "SVONavDataRenderingComponent.h"
#include "SVONavigationDataChunk.h"
#include "SVONavigationDataGenerator.h"
#include "SVONavigationSettings.h"
#include "SVOVersion.h"

#include <AI/NavDataGenerator.h>
#include <DrawDebugHelpers.h>
#include <EngineUtils.h>
#include <NavMesh/NavMeshPath.h>
#include <NavigationSystem.h>

#if WITH_EDITOR
#include <ObjectEditorUtils.h>
#endif

FSVOVolumeNavigationDataDebugInfos::FSVOVolumeNavigationDataDebugInfos() :
    bDebugDrawBounds( false ),
    bDebugDrawNodeAddress( false ),
    bDebugDrawMortonCoords( false ),
    bDebugDrawNodeLocation( false ),
    bDebugDrawLayers( false ),
    LayerIndexToDraw( 0 ),
    bDebugDrawSubNodes( false ),
    bDebugDrawOccludedVoxels( true ),
    bDebugDrawFreeVoxels( false ),
    bDebugDrawActivePaths( false )
{
}

ASVONavigationData::ASVONavigationData() :
    Version( ESVOVersion::Latest )
{
    MaxSimultaneousBoxGenerationJobsCount = 1024;

    if ( !HasAnyFlags( RF_ClassDefaultObject ) )
    {
        FindPathImplementation = FindPath;
        /*FindHierarchicalPathImplementation = FindPath;

        TestPathImplementation = TestPath;
        TestHierarchicalPathImplementation = TestHierarchicalPath;*/

        // RaycastImplementation = NavMeshRaycast;

        // RecastNavMeshImpl = new FPImplRecastNavMesh( this );

        //// add predefined areas up front
        // SupportedAreas.Add( FSupportedAreaData( UNavArea_Null::StaticClass(), RECAST_NULL_AREA ) );
        // SupportedAreas.Add( FSupportedAreaData( UNavArea_LowHeight::StaticClass(), RECAST_LOW_AREA ) );
        // SupportedAreas.Add( FSupportedAreaData( UNavArea_Default::StaticClass(), RECAST_DEFAULT_AREA ) );
    }
}

void ASVONavigationData::PostInitProperties()
{
    if ( UWorld * world = GetWorld() )
    {
        if ( auto * settings = GetDefault< USVONavigationSettings >() )
        {
            if ( HasAnyFlags( RF_NeedLoad )                                                                              //  was loaded
                 && FNavigationSystem::ShouldDiscardSubLevelNavData( *this ) && GEngine->IsSettingUpPlayWorld() == false // this is a @HACK
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

    if ( archive.IsSaving() )
    {
        // When saving, don't serialize the whole VolumeNavigationData array as it may contain navigation data from chunks added by streaming levels
        TArray< FSVOVolumeNavigationData > level_volume_navigation_data;

        if ( SupportsStreaming() && FNavigationSystem::GetCurrent< const UNavigationSystemV1 >( GetWorld() ) != nullptr )
        {
            const auto & level_navigable_bounds = GetNavigableBoundsInLevel( GetLevel() );

            TArray< bool > navigation_data_indices_to_keep;
            navigation_data_indices_to_keep.SetNum( VolumeNavigationData.Num() );

            for ( const auto & navigable_bounds : level_navigable_bounds )
            {
                const auto index = VolumeNavigationData.IndexOfByPredicate( [ &navigable_bounds ]( const auto & navigation_data ) {
                    return !navigation_data.IsInNavigationDataChunk() && !( navigation_data.GetVolumeBounds() == navigable_bounds );
                } );

                if ( index != INDEX_NONE )
                {
                    navigation_data_indices_to_keep[ index ] = true;
                }
            }

            for ( auto index = VolumeNavigationData.Num() - 1; index >= 0; --index )
            {
                if ( navigation_data_indices_to_keep[ index ] )
                {
                    level_volume_navigation_data.Add( VolumeNavigationData[ index ] );
                }
            }
        }
        else
        {
            level_volume_navigation_data = VolumeNavigationData;
        }

        auto volume_count = level_volume_navigation_data.Num();
        archive << volume_count;

        for ( auto index = 0; index < volume_count; index++ )
        {
            level_volume_navigation_data[ index ].Serialize( archive, Version );
        }
    }
    else if ( archive.IsLoading() )
    {
        auto volume_count = VolumeNavigationData.Num();
        archive << volume_count;

        VolumeNavigationData.Reset( volume_count );
        VolumeNavigationData.SetNum( volume_count );

        for ( auto index = 0; index < volume_count; index++ )
        {
            VolumeNavigationData[ index ].Serialize( archive, Version );
        }
    }

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
    const auto needs_rebuild = VolumeNavigationData.FindByPredicate( []( const FSVOVolumeNavigationData & data ) {
        return !data.GetData().IsValid();
    } ) != nullptr;

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

bool ASVONavigationData::SupportsRuntimeGeneration() const
{
    // :TODO:
    return false;
}

bool ASVONavigationData::SupportsStreaming() const
{
    return ( RuntimeGeneration != ERuntimeGenerationType::Dynamic );
}

FNavLocation ASVONavigationData::GetRandomPoint( FSharedConstNavQueryFilter /*filter*/, const UObject * /*querier*/ ) const
{
    FNavLocation result;

    const auto navigation_bounds_num = VolumeNavigationData.Num();

    if ( navigation_bounds_num == 0 )
    {
        return result;
    }

    TArray< int > navigation_bounds_indices;
    navigation_bounds_indices.Reserve( VolumeNavigationData.Num() );

    for ( auto index = 0; index < navigation_bounds_num; index++ )
    {
        navigation_bounds_indices.Add( index );
    }

    // Shuffle the array
    for ( int index = navigation_bounds_indices.Num() - 1; index > 0; --index )
    {
        const auto new_index = FMath::RandRange( 0, index );
        Swap( navigation_bounds_indices[ index ], navigation_bounds_indices[ new_index ] );
    }

    do
    {
        const auto index = navigation_bounds_indices.Pop( false );
        const auto & volume_navigation_data = VolumeNavigationData[ index ];

        const auto random_point = volume_navigation_data.GetRandomPoint();
        if ( random_point.IsSet() )
        {
            result = random_point.GetValue();
            break;
        }
    } while ( navigation_bounds_indices.Num() > 0 );

    return result;
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

ENavigationQueryResult::Type ASVONavigationData::CalcPathCost( const FVector & path_start, const FVector & path_end, float & out_path_cost, const FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    float path_length = 0.f;
    return CalcPathLengthAndCost( path_start, path_end, path_length, out_path_cost, filter, querier );
}

ENavigationQueryResult::Type ASVONavigationData::CalcPathLength( const FVector & path_start, const FVector & path_end, float & out_path_length, const FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    float path_cost = 0.f;
    return CalcPathLengthAndCost( path_start, path_end, out_path_length, path_cost, filter, querier );
}

ENavigationQueryResult::Type ASVONavigationData::CalcPathLengthAndCost( const FVector & path_start, const FVector & path_end, float & out_path_length, float & out_path_cost, FSharedConstNavQueryFilter filter, const UObject * querier ) const
{
    ENavigationQueryResult::Type result = ENavigationQueryResult::Invalid;

    if ( ( path_start - path_end ).IsNearlyZero() )
    {
        out_path_length = 0.f;
        return ENavigationQueryResult::Success;
    }

    auto * volume_navigation_data = GetVolumeNavigationDataContainingPoints( { path_start, path_end } );

    if ( volume_navigation_data == nullptr )
    {
        return ENavigationQueryResult::Error;
    }

    const TSharedRef< FSVONavigationPath > navigation_path = MakeShareable( new FSVONavigationPath() );

    result = FSVOPathFinder::GetPath( navigation_path.Get(), *this, path_start, path_end, filter );

    if ( result == ENavigationQueryResult::Success || ( result == ENavigationQueryResult::Fail && navigation_path->IsPartial() ) )
    {
        out_path_length = navigation_path->GetLength();
        out_path_cost = navigation_path->GetCost();
    }

    return result;
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

void ASVONavigationData::OnStreamingLevelAdded( ULevel * level, UWorld * /*world*/ )
{
    // QUICK_SCOPE_CYCLE_COUNTER( STAT_RecastNavMesh_OnStreamingLevelAdded );

    if ( SupportsStreaming() )
    {
        if ( USVONavigationDataChunk * navigation_data_chunk = GetNavigationDataChunk( level ) )
        {
            for ( const auto & chunk_nav_data : navigation_data_chunk->NavigationData )
            {
                if ( VolumeNavigationData.FindByPredicate( [ &chunk_nav_data ]( const auto & navigation_data ) {
                         return chunk_nav_data.GetVolumeBounds() == navigation_data.GetVolumeBounds();
                     } ) == nullptr )
                {
                    VolumeNavigationData.Add( chunk_nav_data );
                }
            }

            RequestDrawingUpdate();
        }
    }
}

void ASVONavigationData::OnStreamingLevelRemoved( ULevel * level, UWorld * /*world*/ )
{
    // QUICK_SCOPE_CYCLE_COUNTER( STAT_RecastNavMesh_OnStreamingLevelRemoved );

    if ( SupportsStreaming() )
    {
        if ( USVONavigationDataChunk * navigation_data_chunk = GetNavigationDataChunk( level ) )
        {
            for ( const auto & chunk_nav_data : navigation_data_chunk->NavigationData )
            {
                VolumeNavigationData.RemoveAllSwap( [ &chunk_nav_data ]( const auto & nav_data ) {
                    return chunk_nav_data.GetVolumeBounds() == nav_data.GetVolumeBounds();
                } );
            }

            RequestDrawingUpdate();
        }
    }
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

bool ASVONavigationData::IsNodeRefValid( const NavNodeRef node_ref ) const
{
    return FSVONodeAddress( node_ref ).IsValid();
}

void ASVONavigationData::TickActor( const float delta_time, const ELevelTick tick, FActorTickFunction & this_tick_function )
{
    Super::TickActor( delta_time, tick, this_tick_function );

    if ( bEnableDrawing && DebugInfos.bDebugDrawActivePaths )
    {
        for ( auto active_path : ActivePaths )
        {
            if ( active_path.IsValid() )
            {
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
    }
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
                if ( !HasAnyFlags( RF_ClassDefaultObject ) && settings->bNavigationAutoUpdateEnabled )
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

FBox ASVONavigationData::GetBoundingBox() const
{
    FBox bounding_box( ForceInit );

    for ( const auto & bounds : VolumeNavigationData )
    {
        bounding_box += bounds.GetData().GetNavigationBounds();
    }

    return bounding_box;
}

void ASVONavigationData::RemoveDataInBounds( const FBox & bounds )
{
    VolumeNavigationData.RemoveAll( [ &bounds ]( const FSVOVolumeNavigationData & data ) {
        return data.GetVolumeBounds() == bounds;
    } );
}

void ASVONavigationData::AddVolumeNavigationData( FSVOVolumeNavigationData data )
{
    for ( TActorIterator< ASVOBoundsVolume > iterator( GetWorld(), ASVOBoundsVolume::StaticClass() ); iterator; ++iterator )
    {
        const auto * volume = *iterator;

        if ( volume->GetComponentsBoundingBox( true ) == data.GetVolumeBounds() )
        {
            data.SetVolumeNavigationQueryFilter( volume->GetVolumeNavigationQueryFilter() );
            break;
        }
    }

    VolumeNavigationData.Emplace( MoveTemp( data ) );
}

const FSVOVolumeNavigationData * ASVONavigationData::GetVolumeNavigationDataContainingPoints( const TArray< FVector > & points ) const
{
    return VolumeNavigationData.FindByPredicate( [ this, &points ]( const FSVOVolumeNavigationData & data ) {
        const auto & bounds = data.GetData().GetNavigationBounds();
        for ( const auto & point : points )
        {
            if ( !bounds.IsInside( point ) )
            {
                return false;
            }
        }
        return true;
    } );
}

void ASVONavigationData::UpdateNavVersion()
{
    Version = ESVOVersion::Latest;
}

void ASVONavigationData::RecreateDefaultFilter() const
{
    DefaultQueryFilter->SetFilterType< FSVONavigationQueryFilterImpl >();
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

void ASVONavigationData::OnNavigationDataUpdatedInBounds( const TArray< FBox > & updated_bounds )
{
    InvalidateAffectedPaths( updated_bounds );
}

void ASVONavigationData::ClearNavigationData()
{
    VolumeNavigationData.Reset();
    RequestDrawingUpdate();
}

void ASVONavigationData::BuildNavigationData()
{
    RebuildAll();
}

void ASVONavigationData::InvalidateAffectedPaths( const TArray< FBox > & updated_bounds )
{
    const int32 paths_count = ActivePaths.Num();
    const int32 updated_bounds_count = updated_bounds.Num();

    if ( updated_bounds_count == 0 || paths_count == 0 )
    {
        return;
    }

    // Paths can be registered from async pathfinding thread.
    // Theoretically paths are invalidated synchronously by the navigation system
    // before starting async queries task but protecting ActivePaths will make
    // the system safer in case of future timing changes.
    {
        FScopeLock path_lock( &ActivePathsLock );

        FNavPathWeakPtr * weak_path_ptr = ( ActivePaths.GetData() + paths_count - 1 );

        for ( int32 path_index = paths_count - 1; path_index >= 0; --path_index, --weak_path_ptr )
        {
            FNavPathSharedPtr shared_path = weak_path_ptr->Pin();
            if ( !weak_path_ptr->IsValid() )
            {
                ActivePaths.RemoveAtSwap( path_index, 1, /*bAllowShrinking=*/false );
            }
            else
            {
                const FNavigationPath * path = shared_path.Get();
                if ( !path->IsReady() || path->GetIgnoreInvalidation() )
                {
                    // path not filled yet or doesn't care about invalidation
                    continue;
                }

                for ( const auto & path_point : path->GetPathPoints() )
                {
                    if ( updated_bounds.FindByPredicate( [ &path_point ]( const FBox & bounds ) {
                             return bounds.IsInside( path_point.Location );
                         } ) != nullptr )
                    {
                        shared_path->Invalidate();
                        ActivePaths.RemoveAtSwap( path_index, 1, /*bAllowShrinking=*/false );

                        break;
                    }
                }

                if ( !shared_path->IsValid() )
                {
                    break;
                }
            }
        }
    }
}

void ASVONavigationData::OnNavigationDataGenerationFinished()
{
    if ( UWorld * world = GetWorld() )
    {
        if ( !world->IsPendingKill() )
        {
#if WITH_EDITOR
            // For navmeshes that support streaming create navigation data holders in each streaming level
            // so parts of navmesh can be streamed in/out with those levels
            if ( !world->IsGameWorld() )
            {
                const auto & levels = world->GetLevels();

                for ( auto * level : levels )
                {
                    if ( level->IsPersistentLevel() )
                    {
                        continue;
                    }

                    USVONavigationDataChunk * navigation_data_chunk = GetNavigationDataChunk( level );

                    if ( SupportsStreaming() )
                    {
                        // We use navigation volumes that belongs to this streaming level to find tiles we want to save
                        const auto & level_nav_bounds = GetNavigableBoundsInLevel( level );

                        TArray< int32 > navigation_data_indices;
                        navigation_data_indices.Reserve( level_nav_bounds.Num() );

                        for ( const auto & nav_bounds : level_nav_bounds )
                        {
                            const auto index = VolumeNavigationData.IndexOfByPredicate( [ &nav_bounds ]( const FSVOVolumeNavigationData & data ) {
                                const auto & bounds = data.GetData().GetVolumeBounds();
                                return bounds == nav_bounds;
                            } );

                            if ( index != INDEX_NONE )
                            {
                                navigation_data_indices.Add( index );
                            }
                        }

                        if ( navigation_data_indices.Num() > 0 )
                        {
                            // Create new chunk only if we have something to save in it
                            if ( navigation_data_chunk == nullptr )
                            {
                                navigation_data_chunk = NewObject< USVONavigationDataChunk >( level );
                                navigation_data_chunk->NavigationDataName = GetFName();
                                level->NavDataChunks.Add( navigation_data_chunk );
                            }

                            for ( const auto index : navigation_data_indices )
                            {
                                navigation_data_chunk->NavigationData.Emplace( VolumeNavigationData[ index ] );
                                VolumeNavigationData.RemoveAtSwap( index );
                            }

                            navigation_data_chunk->MarkPackageDirty();
                            continue;
                        }
                    }

                    // stale data that is left in the level
                    if ( navigation_data_chunk != nullptr )
                    {
                        navigation_data_chunk->ReleaseNavigationData();
                        navigation_data_chunk->MarkPackageDirty();
                        level->NavDataChunks.Remove( navigation_data_chunk );
                    }
                }
            }

            // force navmesh drawing update
            RequestDrawingUpdate( /*bForce=*/true );
#endif // WITH_EDITOR

            UNavigationSystemV1 * NavSys = FNavigationSystem::GetCurrent< UNavigationSystemV1 >( world );
            if ( NavSys )
            {
                NavSys->OnNavigationGenerationFinished( *this );
            }
        }
    }
}

USVONavigationDataChunk * ASVONavigationData::GetNavigationDataChunk( ULevel * level ) const
{
    const auto this_name = GetFName();

    if ( const auto * result = level->NavDataChunks.FindByPredicate( [ & ]( const UNavigationDataChunk * chunk ) {
             return chunk->NavigationDataName == this_name;
         } ) )
    {
        return Cast< USVONavigationDataChunk >( *result );
    }

    return nullptr;
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
    FSVONavigationPath * svo_navigation_path = navigation_path != nullptr
                                                   ? navigation_path->CastPath< FSVONavigationPath >()
                                                   : nullptr;

    if ( svo_navigation_path != nullptr )
    {
        result.Path = path_finding_query.PathInstanceToFill;
        svo_navigation_path->ResetForRepath();
    }
    else
    {
        result.Path = self->CreatePathInstance< FSVONavigationPath >( path_finding_query );
        navigation_path = result.Path.Get();
        svo_navigation_path = navigation_path != nullptr
                                  ? navigation_path->CastPath< FSVONavigationPath >()
                                  : nullptr;
    }

    if ( navigation_path != nullptr )
    {
        if ( path_finding_query.QueryFilter.IsValid() )
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
                result.Result = FSVOPathFinder::GetPath( *svo_navigation_path, *self, path_finding_query.StartLocation, adjusted_end_location, path_finding_query.QueryFilter );
            }
        }
    }

    return result;
}

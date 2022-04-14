#pragma once

#include "SVONavigationTypes.h"
#include "SVOVolumeNavigationData.h"

#include <CoreMinimal.h>
#include <NavigationData.h>

#include "SVONavigationData.generated.h"

class USVONavigationDataChunk;
class USVONavDataRenderingComponent;
struct FSVONavigationBounds;

USTRUCT()
struct SVONAVIGATION_API FSVOVolumeNavigationDataDebugInfos
{
    GENERATED_USTRUCT_BODY()

    FSVOVolumeNavigationDataDebugInfos();

    UPROPERTY( EditInstanceOnly )
    uint8 bDebugDrawBounds : 1;

    UPROPERTY( EditInstanceOnly )
    uint8 bDebugDrawNodeAddress : 1;

    UPROPERTY( EditInstanceOnly )
    uint8 bDebugDrawMortonCoords : 1;

    UPROPERTY( EditInstanceOnly )
    uint8 bDebugDrawNodeLocation : 1;

    UPROPERTY( EditInstanceOnly )
    uint8 bDebugDrawLayers : 1;

    UPROPERTY( EditInstanceOnly, meta = ( EditCondition = "bDebugDrawLayers", ClampMin = "0", UIMin = "0" ) )
    uint8 LayerIndexToDraw;

    UPROPERTY( EditInstanceOnly )
    uint8 bDebugDrawSubNodes : 1;

    UPROPERTY( EditInstanceOnly )
    uint8 bDebugDrawOccludedVoxels : 1;

    UPROPERTY( EditInstanceOnly )
    uint8 bDebugDrawFreeVoxels : 1;

    UPROPERTY( EditInstanceOnly )
    uint8 bDebugDrawActivePaths : 1;
};

UCLASS( config = Engine, defaultconfig, hidecategories = ( Input, Physics, Collisions, Lighting, Rendering, Tags, "Utilities|Transformation", Actor, Layers, Replication ), notplaceable )
class SVONAVIGATION_API ASVONavigationData final : public ANavigationData
{
    GENERATED_BODY()

public:
    ASVONavigationData();

    friend class FSVONavigationDataGenerator;

    const FSVOVolumeNavigationDataDebugInfos & GetDebugInfos() const;
    const TArray< FSVOVolumeNavigationData > & GetVolumeNavigationData() const;

    void PostInitProperties() override;
    void PostLoad() override;
    void Serialize( FArchive & archive ) override;
    void CleanUp() override;
    bool NeedsRebuild() const override;
    void EnsureBuildCompletion() override;
    bool SupportsRuntimeGeneration() const override;
    bool SupportsStreaming() const override;
    FNavLocation GetRandomPoint( FSharedConstNavQueryFilter filter, const UObject * querier ) const override;
    bool GetRandomReachablePointInRadius( const FVector & origin, float radius, FNavLocation & out_result, FSharedConstNavQueryFilter filter = nullptr, const UObject * querier = nullptr ) const override;
    bool GetRandomPointInNavigableRadius( const FVector & origin, float radius, FNavLocation & out_result, FSharedConstNavQueryFilter filter = nullptr, const UObject * querier = nullptr ) const override;
    void BatchRaycast( TArray< FNavigationRaycastWork > & workload, FSharedConstNavQueryFilter filter, const UObject * querier = nullptr ) const override;
    bool FindMoveAlongSurface( const FNavLocation & start_location, const FVector & target_position, FNavLocation & out_location, FSharedConstNavQueryFilter filter = nullptr, const UObject * querier = nullptr ) const override;
    bool ProjectPoint( const FVector & point, FNavLocation & out_location, const FVector & extent, FSharedConstNavQueryFilter filter = nullptr, const UObject * querier = nullptr ) const override;
    void BatchProjectPoints( TArray< FNavigationProjectionWork > & Workload, const FVector & Extent, FSharedConstNavQueryFilter Filter = nullptr, const UObject * Querier = nullptr ) const override;
    void BatchProjectPoints( TArray< FNavigationProjectionWork > & Workload, FSharedConstNavQueryFilter Filter = nullptr, const UObject * Querier = nullptr ) const override;
    ENavigationQueryResult::Type CalcPathCost( const FVector & path_start, const FVector & path_end, float & out_path_cost, FSharedConstNavQueryFilter filter = nullptr, const UObject * querier = nullptr ) const override;
    ENavigationQueryResult::Type CalcPathLength( const FVector & path_start, const FVector & path_end, float & out_path_length, FSharedConstNavQueryFilter filter = nullptr, const UObject * querier = nullptr ) const override;
    ENavigationQueryResult::Type CalcPathLengthAndCost( const FVector & path_start, const FVector & path_end, float & out_path_length, float & out_path_cost, FSharedConstNavQueryFilter filter = nullptr, const UObject * querier = nullptr ) const override;
    bool DoesNodeContainLocation( NavNodeRef node_ref, const FVector & world_space_location ) const override;
    UPrimitiveComponent * ConstructRenderingComponent() override;
    void OnStreamingLevelAdded( ULevel * level, UWorld * world ) override;
    void OnStreamingLevelRemoved( ULevel * level, UWorld * world ) override;
    void OnNavAreaChanged() override;
    void OnNavAreaAdded( const UClass * nav_area_class, int32 agent_index ) override;
    int32 GetNewAreaID( const UClass * nav_area_class ) const override;
    int32 GetMaxSupportedAreas() const override;
    bool IsNodeRefValid( NavNodeRef node_ref ) const override;
    void TickActor( float delta_time, ELevelTick tick_type, FActorTickFunction & this_tick_function ) override;

#if WITH_EDITOR
    void PostEditChangeProperty( FPropertyChangedEvent & property_changed_event ) override;
    bool ShouldExport() override;
#endif

#if !UE_BUILD_SHIPPING
    uint32 LogMemUsed() const override;
#endif

    void ConditionalConstructGenerator() override;

    void RequestDrawingUpdate( bool force = false );
    FBox GetBoundingBox() const;
    void RemoveDataInBounds( const FBox & bounds );

    template< typename _ALLOCATOR_TYPE_ >
    void RemoveDataInBounds( const TArray< FBox, _ALLOCATOR_TYPE_ > & bounds_array )
    {
        for ( const auto & bounds : bounds_array )
        {
            RemoveDataInBounds( bounds );
        }
    }

    void AddVolumeNavigationData( FSVOVolumeNavigationData data );
    const FSVOVolumeNavigationData * GetVolumeNavigationDataContainingPoints( const TArray< FVector > & points ) const;
    void UpdateNavVersion();

private:
    void RecreateDefaultFilter() const;
    void UpdateDrawing() const;
    void ResetGenerator( bool cancel_build = true );
    void OnNavigationDataUpdatedInBounds( const TArray< FBox > & updated_bounds );

    UFUNCTION( CallInEditor )
    void ClearNavigationData();

    UFUNCTION( CallInEditor )
    void BuildNavigationData();

    void InvalidateAffectedPaths( const TArray< FBox > & updated_bounds );
    void OnNavigationDataGenerationFinished();
    USVONavigationDataChunk * GetNavigationDataChunk( ULevel * level ) const;

    static FPathFindingResult FindPath( const FNavAgentProperties & agent_properties, const FPathFindingQuery & path_finding_query );

    UPROPERTY( EditAnywhere, config, Category = "Display" )
    FSVOVolumeNavigationDataDebugInfos DebugInfos;

    UPROPERTY( EditAnywhere, config, Category = "Generation" )
    FSVODataGenerationSettings GenerationSettings;

    UPROPERTY( EditAnywhere, Category = "Generation", config, meta = ( ClampMin = "0", UIMin = "0" ), AdvancedDisplay )
    int32 MaxSimultaneousBoxGenerationJobsCount;

    TArray< FSVOVolumeNavigationData > VolumeNavigationData;
    ESVOVersion Version;
};

FORCEINLINE const TArray< FSVOVolumeNavigationData > & ASVONavigationData::GetVolumeNavigationData() const
{
    return VolumeNavigationData;
}

FORCEINLINE const FSVOVolumeNavigationDataDebugInfos & ASVONavigationData::GetDebugInfos() const
{
    return DebugInfos;
}
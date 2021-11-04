#pragma once

#include "SVOBoundsNavigationData.h"

#include <CoreMinimal.h>

#include "SVOData.h"

#include <NavigationData.h>

#include "SVONavigationData.generated.h"

class USVONavDataRenderingComponent;
struct FSVONavigationBounds;

struct FSVONavigationDataBoundsKey
{
    FSVONavigationDataBoundsKey() = default;
    FSVONavigationDataBoundsKey( const FBox & volume_bounds ) :
        VolumeBounds( volume_bounds )
    {}

    FBox VolumeBounds;

    friend uint32 GetTypeHash( const FSVONavigationDataBoundsKey & element )
    {
        return HashCombine( GetTypeHash( element.VolumeBounds.GetCenter() ), GetTypeHash( element.VolumeBounds.GetExtent() ) );
    }

    bool operator==( const FSVONavigationDataBoundsKey & other ) const
    {
        return VolumeBounds == other.VolumeBounds;
    }
};

FORCEINLINE FArchive & operator<<( FArchive & archive, FSVONavigationDataBoundsKey & data )
{
    archive << data.VolumeBounds;
    return archive;
}

UCLASS( config = Engine, defaultconfig, hidecategories = ( Input, Physics, Collisions, Lighting, Rendering, Tags, "Utilities|Transformation", Actor, Layers, Replication ), notplaceable )
class SVONAVIGATION_API ASVONavigationData final : public ANavigationData
{
    GENERATED_BODY()

public:
    ASVONavigationData();

    friend class FSVONavigationDataGenerator;

    const FSVONavigationBoundsDataDebugInfos & GetDebugInfos() const;
    const FSVOData & GetSVOData() const;

    void PostInitProperties() override;
    void PostLoad() override;
    void Serialize( FArchive & archive ) override;
    void CleanUp() override;

    void EnsureBuildCompletion() override;
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

#if WITH_EDITOR
    void PostEditChangeProperty( FPropertyChangedEvent & property_changed_event ) override;
    bool ShouldExport() override;
#endif

#if !UE_BUILD_SHIPPING
    uint32 LogMemUsed() const override;
#endif

    void ConditionalConstructGenerator() override;

    void RequestDrawingUpdate( bool force = false );

private:
    void RecreateDefaultFilter();
    void UpdateDrawing();
    void ResetGenerator( bool cancel_build = true );
    void OnNavigationDataUpdatedInBounds( const TArray< FBox > & updated_boxes );

    UFUNCTION( CallInEditor )
    void ClearNavigationData();

    UFUNCTION( CallInEditor )
    void BuildNavigationData();

    static FPathFindingResult FindPath( const FNavAgentProperties & agent_properties, const FPathFindingQuery & path_finding_query );

    FUniqueSVODataPtr SVODataPtr;

    UPROPERTY( EditAnywhere, config, Category = "Display" )
    FSVONavigationBoundsDataDebugInfos DebugInfos;

    UPROPERTY( EditAnywhere, config, Category = "Generation" )
    FSVODataGenerationSettings GenerationSettings;

    UPROPERTY( EditAnywhere, Category = "Generation", config, meta = ( ClampMin = "0", UIMin = "0" ), AdvancedDisplay )
    int32 MaxSimultaneousBoxGenerationJobsCount;

    uint32 SVOVersion;
};

FORCEINLINE const FSVONavigationBoundsDataDebugInfos & ASVONavigationData::GetDebugInfos() const
{
    return DebugInfos;
}

FORCEINLINE const FSVOData & ASVONavigationData::GetSVOData() const
{
    return *SVODataPtr.Get();
}
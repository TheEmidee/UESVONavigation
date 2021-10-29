#pragma once

#include "Chaos/AABB.h"
#include "GraphAStar.h"
#include "SVONavigationTypes.h"

#include <NavigationData.h>

#include "SVOPathFindingAlgorithm.generated.h"

struct FSVOLinkWithCost;
class FSVOBoundsNavigationData;
class USVOPathCostCalculator;
class USVOPathHeuristicCalculator;
class FSVONavigationQueryFilterImpl;
class ASVONavigationData;

struct FSVOLinkWithLocation
{
    FSVOLinkWithLocation() = default;
    FSVOLinkWithLocation( const FSVOOctreeLink & link, const FVector & location );
    FSVOLinkWithLocation( const FSVOOctreeLink & link, const FSVOBoundsNavigationData & bounds_navigation_data );

    bool operator==( const FSVOLinkWithLocation & other ) const
    {
        return Link == other.Link;
    }

    bool operator!=( const FSVOLinkWithLocation & other ) const
    {
        return !operator==( other );
    }

    FSVOOctreeLink Link;
    FVector Location;
};

FORCEINLINE uint32 GetTypeHash( const FSVOLinkWithLocation & link_with_location )
{
    return GetTypeHash( link_with_location.Link );
}

struct FSVOPathFinderDebugNodeCost
{
    FSVOPathFinderDebugNodeCost() = default;
    FSVOPathFinderDebugNodeCost( const FSVOLinkWithLocation & from, const FSVOLinkWithLocation & to, const float cost, const bool is_closed );

    void Reset();

    FSVOLinkWithLocation From;
    FSVOLinkWithLocation To;
    float Cost;
    uint8 bIsClosed : 1;
};

USTRUCT()
struct SVONAVIGATION_API FSVOPathFinderDebugInfos
{
    GENERATED_USTRUCT_BODY()

    void Reset();

    FSVOPathFinderDebugNodeCost LastLastProcessedSingleNode;
    TArray< FSVOPathFinderDebugNodeCost > ProcessedNeighbors;
    FNavigationPath CurrentBestPath;

    UPROPERTY( VisibleAnywhere )
    int Iterations;

    UPROPERTY( VisibleAnywhere )
    int VisitedNodes;

    UPROPERTY( VisibleAnywhere )
    int PathSegmentCount;

    UPROPERTY( VisibleAnywhere )
    float PathLength;
};

struct FSVOPathFindingParameters
{
    FSVOPathFindingParameters( const FNavAgentProperties & agent_properties, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query );

    const FNavAgentProperties & AgentProperties;
    const ASVONavigationData & NavigationData;
    FVector StartLocation;
    FVector EndLocation;
    const FNavigationQueryFilter & NavigationQueryFilter;
    const FSVONavigationQueryFilterImpl * QueryFilterImplementation;
    const FSVONavigationQueryFilterSettings & QueryFilterSettings;
    const USVOPathHeuristicCalculator * HeuristicCalculator;
    const USVOPathCostCalculator * CostCalculator;
    const FSVOBoundsNavigationData * BoundsNavigationData;
    FSVOOctreeLink StartLink;
    FSVOOctreeLink EndLink;
    float VerticalOffset;
};

enum class ESVOPathFindingAlgorithmState : uint8
{
    Init,
    ProcessNode,
    ProcessNeighbor,
    Ended
};

enum class ESVOPathFindingAlgorithmStepperStatus : uint8
{
    MustContinue,
    IsStopped
};

class FSVOPathFindingAlgorithmStepper;

class FSVOPathFindingAlgorithmObserver
{
public:
    explicit FSVOPathFindingAlgorithmObserver( const FSVOPathFindingAlgorithmStepper & stepper );

    virtual ~FSVOPathFindingAlgorithmObserver() = default;

    virtual void OnProcessSingleNode( const FGraphAStarDefaultNode<FSVOBoundsNavigationData> & node ) {}
    virtual void OnProcessNeighbor( const FGraphAStarDefaultNode<FSVOBoundsNavigationData> & parent, const FGraphAStarDefaultNode<FSVOBoundsNavigationData> & neighbor, const float cost ) {}
    virtual void OnProcessNeighbor( const FGraphAStarDefaultNode<FSVOBoundsNavigationData> & neighbor ) {}
    virtual void OnSearchSuccess( const TArray< FSVOOctreeLink > & link_path ) {}

protected:
    const FSVOPathFindingAlgorithmStepper & Stepper;
};

class FSVOPathFindingAlgorithmStepper : protected FGraphAStar< FSVOBoundsNavigationData, FGraphAStarDefaultPolicy, FGraphAStarDefaultNode< FSVOBoundsNavigationData > >
{
public:
    explicit FSVOPathFindingAlgorithmStepper( const FSVOPathFindingParameters & parameters );

    ESVOPathFindingAlgorithmState GetState() const;
    const FSVOPathFindingParameters & GetParameters() const;
    ESVOPathFindingAlgorithmStepperStatus Step( EGraphAStarResult & result );
    void AddObserver( TSharedPtr< FSVOPathFindingAlgorithmObserver > observer );

protected:
    ESVOPathFindingAlgorithmStepperStatus Init( EGraphAStarResult & result );
    ESVOPathFindingAlgorithmStepperStatus ProcessSingleNode( EGraphAStarResult & result );
    virtual ESVOPathFindingAlgorithmStepperStatus ProcessNeighbor();
    ESVOPathFindingAlgorithmStepperStatus Ended( EGraphAStarResult & result );

    void SetState( ESVOPathFindingAlgorithmState new_state );

    struct NeighborIndexIncrement
    {
        NeighborIndexIncrement( TArray< FSVOOctreeLink > & neighbors, int & neighbor_index, ESVOPathFindingAlgorithmState & state );
        ~NeighborIndexIncrement();

        TArray< FSVOOctreeLink > & Neighbors;
        int & NeighborIndex;
        ESVOPathFindingAlgorithmState & State;
    };

    ESVOPathFindingAlgorithmState State;
    FSVOPathFindingParameters Parameters;
    int32 ConsideredNodeIndex;
    int32 BestNodeIndex;
    float BestNodeCost;
    int NeighborIndex;
    TArray< TSharedPtr< FSVOPathFindingAlgorithmObserver > > Observers;
    TArray< FSVOOctreeLink > Neighbors;
};

FORCEINLINE ESVOPathFindingAlgorithmState FSVOPathFindingAlgorithmStepper::GetState() const
{
    return State;
}

FORCEINLINE const FSVOPathFindingParameters & FSVOPathFindingAlgorithmStepper::GetParameters() const
{
    return Parameters;
}

USTRUCT()
struct SVONAVIGATION_API FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters
{
    GENERATED_USTRUCT_BODY()

    FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters() :
        AgentRadiusMultiplier( 1.0f ),
        bShowLineOfSightTraces( false ),
        TraceType( ETraceTypeQuery::TraceTypeQuery1 )
    {}

    UPROPERTY( EditAnywhere )
    float AgentRadiusMultiplier;

    UPROPERTY( EditAnywhere )
    uint8 bShowLineOfSightTraces : 1;

    UPROPERTY( EditAnywhere )
    TEnumAsByte< ETraceTypeQuery > TraceType;
};

class FSVOPathFindingAlgorithmStepper_ThetaStar final : public FSVOPathFindingAlgorithmStepper
{
public:
    FSVOPathFindingAlgorithmStepper_ThetaStar( const FSVOPathFindingParameters & parameters, const FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters & theta_star_parameters );

protected:

    ESVOPathFindingAlgorithmStepperStatus ProcessNeighbor() override;
    bool HasLineOfSight( FSVOOctreeLink from, FSVOOctreeLink to ) const;

    const FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters & ThetaStarParameters;
};

class FSVOPathFindingAStarObserver_BuildPath final : public FSVOPathFindingAlgorithmObserver
{
public:
    FSVOPathFindingAStarObserver_BuildPath( FNavigationPath & navigation_path, const FSVOPathFindingAlgorithmStepper & stepper );

    void OnSearchSuccess( const TArray< FSVOOctreeLink > & ) override;

private:
    FNavigationPath & NavigationPath;
};

class FSVOPathFindingAStarObserver_GenerateDebugInfos final : public FSVOPathFindingAlgorithmObserver
{
public:
    FSVOPathFindingAStarObserver_GenerateDebugInfos( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingAlgorithmStepper & stepper );

    void OnProcessSingleNode( const FGraphAStarDefaultNode<FSVOBoundsNavigationData> & node ) override;
    void OnProcessNeighbor( const FGraphAStarDefaultNode<FSVOBoundsNavigationData> & parent, const FGraphAStarDefaultNode<FSVOBoundsNavigationData> & neighbor, const float cost ) override;
    void OnProcessNeighbor( const FGraphAStarDefaultNode<FSVOBoundsNavigationData> & neighbor ) override;
    void OnSearchSuccess( const TArray< FSVOOctreeLink > & ) override;
private:
    FSVOPathFinderDebugInfos & DebugInfos;
};

UCLASS( HideDropdown, NotBlueprintable )
class SVONAVIGATION_API USVOPathFindingAlgorithm : public UObject
{
    GENERATED_BODY()

public:
    virtual ENavigationQueryResult::Type GetPath( FNavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const;
    virtual TSharedPtr< FSVOPathFindingAlgorithmStepper > GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const;
};

UCLASS()
class SVONAVIGATION_API USVOPathFindingAlgorithmAStar final : public USVOPathFindingAlgorithm
{
    GENERATED_BODY()

public:
    ENavigationQueryResult::Type GetPath( FNavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const override;
    TSharedPtr< FSVOPathFindingAlgorithmStepper > GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const override;
};

UCLASS( Blueprintable )
class SVONAVIGATION_API USVOPathFindingAlgorithmThetaStar final : public USVOPathFindingAlgorithm
{
    GENERATED_BODY()

public:
    ENavigationQueryResult::Type GetPath( FNavigationPath & navigation_path, const FSVOPathFindingParameters & params ) const override;
    TSharedPtr< FSVOPathFindingAlgorithmStepper > GetDebugPathStepper( FSVOPathFinderDebugInfos & debug_infos, const FSVOPathFindingParameters params ) const override;

private:
    UPROPERTY( EditAnywhere )
    FSVOPathFindingAlgorithmStepper_ThetaStar_Parameters ThetaStarParameters;
};
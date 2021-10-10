#pragma once

#include "SVONavigationTypes.h"

#include <NavigationData.h>

#include "SVOPathFindingAlgorithm.generated.h"

class FSVOBoundsNavigationData;
class USVOPathCostCalculator;
class USVOPathHeuristicCalculator;
class FSVONavigationQueryFilterImpl;
class ASVONavigationData;

struct FSVOLinkWithCost
{
    FSVOLinkWithCost( const FSVOOctreeLink & link, float cost ) :
        Link( link ),
        Cost( cost )
    {
    }

    FSVOOctreeLink Link;
    float Cost;

    bool operator<( const FSVOLinkWithCost & other ) const
    {
        return Cost > other.Cost;
    }
};

struct FSVOPathFinderDebugCost
{
    FVector Location = { 0.0f, 0.0f, 0.0f };
    float Cost = -1.0f;
    bool WasEvaluated = false;
};

struct FSVOPathFinderDebugStep
{
    FSVOPathFinderDebugCost CurrentLocationCost;
    TArray< FSVOPathFinderDebugCost, TInlineAllocator< 6 > > NeighborLocationCosts;
};

USTRUCT()
struct SVONAVIGATION_API FSVOPathFinderDebugInfos
{
    GENERATED_USTRUCT_BODY()

    void Reset();

    TArray< FSVOPathFinderDebugStep > DebugSteps;
    FNavigationPath CurrentBestPath;

    UPROPERTY( VisibleAnywhere )
    int Iterations;

    UPROPERTY( VisibleAnywhere )
    int VisitedNodes;
};

struct SVONAVIGATION_API FSVOPathFindingAlgorithmStepper
{
    FSVOPathFindingAlgorithmStepper( FNavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query );
    virtual ~FSVOPathFindingAlgorithmStepper() = default;

    FNavigationPath & GetNavigationPath() const;
    const FSVOBoundsNavigationData * GetBoundsNavigationData() const;
    FVector GetStartLocation() const;
    FVector GetEndLocation() const;    
    FSVOOctreeLink GetStartLink() const;
    FSVOOctreeLink GetEndLink() const;
    float GetVerticalOffset() const;
    
    virtual bool Step( ENavigationQueryResult::Type & result )
    {
        return false;
    }
    virtual bool Step( FSVOPathFinderDebugInfos & debug_infos, ENavigationQueryResult::Type & result )
    {
        return false;
    }

protected:
    FNavigationPath & NavigationPath;
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

FORCEINLINE FNavigationPath & FSVOPathFindingAlgorithmStepper::GetNavigationPath() const
{
    return NavigationPath;   
}

FORCEINLINE const FSVOBoundsNavigationData * FSVOPathFindingAlgorithmStepper::GetBoundsNavigationData() const
{
    return BoundsNavigationData;
}

FORCEINLINE FVector FSVOPathFindingAlgorithmStepper::GetStartLocation() const
{
    return StartLocation;
}

FORCEINLINE FVector FSVOPathFindingAlgorithmStepper::GetEndLocation() const
{
    return EndLocation;
}

FORCEINLINE FSVOOctreeLink FSVOPathFindingAlgorithmStepper::GetStartLink() const
{
    return StartLink;
}

FORCEINLINE FSVOOctreeLink FSVOPathFindingAlgorithmStepper::GetEndLink() const
{
    return EndLink;
}

FORCEINLINE float FSVOPathFindingAlgorithmStepper::GetVerticalOffset() const
{
    return VerticalOffset;
}

template< typename _STEPPER_IMPLEMENTATION_TYPE_ >
struct TSVOPathFindingAlgorithmStepper : FSVOPathFindingAlgorithmStepper
{
    TSVOPathFindingAlgorithmStepper( FNavigationPath & Navigation_Path, const ASVONavigationData & Navigation_Data, const FVector & Start_Location, const FVector & End_Location, const FPathFindingQuery & Path_Finding_Query ) :
        FSVOPathFindingAlgorithmStepper( Navigation_Path, Navigation_Data, Start_Location, End_Location, Path_Finding_Query )
    {
    }

    bool Step( ENavigationQueryResult::Type & result ) override;
    bool Step( FSVOPathFinderDebugInfos & debug_infos, ENavigationQueryResult::Type & result ) override;

protected:
    

private:
    _STEPPER_IMPLEMENTATION_TYPE_ StepperImplementation;
};

UCLASS( HideDropdown, NotBlueprintable )
class SVONAVIGATION_API USVOPathFindingAlgorithm : public UObject
{
    GENERATED_BODY()

public:
    virtual ENavigationQueryResult::Type GetPath( FNavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query ) const;
    virtual TSharedPtr< FSVOPathFindingAlgorithmStepper > GetDebugPathStepper( FNavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query );
};

struct SVONAVIGATION_API FSVOPathFindingAlgorithmStepperAStar : public FSVOPathFindingAlgorithmStepper
{
    FSVOPathFindingAlgorithmStepperAStar( FNavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query );

    const TMap< FSVOOctreeLink, FSVOOctreeLink > & GetCameFrom() const;
    
protected:
    TArray< FSVOLinkWithCost > Frontier;
    TMap< FSVOOctreeLink, FSVOOctreeLink > CameFrom;
    TMap< FSVOOctreeLink, float > CostSoFar;
};

FORCEINLINE const TMap< FSVOOctreeLink, FSVOOctreeLink > & FSVOPathFindingAlgorithmStepperAStar::GetCameFrom() const
{
    return CameFrom;
}

template< typename _VISITOR_TYPE_ >
struct SVONAVIGATION_API TSVOPathFindingAlgorithmStepperAStar final : public FSVOPathFindingAlgorithmStepperAStar
{
    bool Step( FSVOPathFinderDebugInfos & debug_infos, ENavigationQueryResult::Type & result ) override
    {
        return false;
    }
    TSVOPathFindingAlgorithmStepperAStar( FNavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query );

    bool Step( ENavigationQueryResult::Type & result ) override;
private:
    _VISITOR_TYPE_ Visitor;
};

struct FSVOPathFindingAlgorithmStepperAStarVisitor
{
    explicit FSVOPathFindingAlgorithmStepperAStarVisitor( FSVOPathFindingAlgorithmStepperAStar & stepper ) :
        Stepper( stepper )
    {}
    
    virtual ~FSVOPathFindingAlgorithmStepperAStarVisitor() = default;

    virtual void OnOpenNode( const FSVOLinkWithCost & link_with_cost ) = 0;
    virtual void OnEndLinkReached() = 0;
    virtual void OnNeighborEvaluated() = 0;
    virtual void OnNeighborExpanded() = 0;
    virtual void OnEndStep() = 0;

protected:    
    FSVOPathFindingAlgorithmStepperAStar & Stepper;
};

struct FSVOPathFindingAlgorithmStepperAStarVisitorDebug : public FSVOPathFindingAlgorithmStepperAStarVisitor
{
    explicit FSVOPathFindingAlgorithmStepperAStarVisitorDebug( FSVOPathFindingAlgorithmStepperAStar & stepper ) :
        FSVOPathFindingAlgorithmStepperAStarVisitor( stepper )
    {}

    void OnOpenNode( const FSVOLinkWithCost & link_with_cost ) override;
    void OnEndLinkReached() override;
    void OnNeighborEvaluated() override;
    void OnNeighborExpanded() override;
    void OnEndStep() override;
};

struct FSVOPathFindingAlgorithmStepperAStarVisitorPathConstructor : public FSVOPathFindingAlgorithmStepperAStarVisitor
{
    explicit FSVOPathFindingAlgorithmStepperAStarVisitorPathConstructor( FSVOPathFindingAlgorithmStepperAStar & stepper ) :
        FSVOPathFindingAlgorithmStepperAStarVisitor( stepper )
    {}

    void OnOpenNode( const FSVOLinkWithCost & link_with_cost ) override;
    void OnEndLinkReached() override;
    void OnNeighborEvaluated() override;
    void OnNeighborExpanded() override;
    void OnEndStep() override;
};

UCLASS()
class SVONAVIGATION_API USVOPathFindingAlgorithmAStar final : public USVOPathFindingAlgorithm
{
    GENERATED_BODY()

public:
    ENavigationQueryResult::Type GetPath( FNavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query ) const override;
    TSharedPtr< FSVOPathFindingAlgorithmStepper > GetDebugPathStepper( FNavigationPath & navigation_path, const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query ) override;
};


//--------------------------------

struct FParams
{
    FParams( const ASVONavigationData & navigation_data, const FVector & start_location, const FVector & end_location, const FPathFindingQuery & path_finding_query );
};

class FPathBuilder
{
public:
    FPathBuilder( const FParams & params ){}

    FNavigationPath navigation_path;
};

class FDebugPath
{
public:
    FDebugPath( const FParams & params ){}

    FSVOPathFinderDebugInfos debug_infos;
};

template< typename _VISITOR_ >
class TAlgoAStar
{
public:
    TAlgoAStar( const FParams & params ) :
        Visitor( params ),
        Params( params )
    {}

    const _VISITOR_ & GetVisitor() const { return Visitor; }

    bool Step( ENavigationQueryResult::Type & result )
    {
        return false;
    }

private:
    _VISITOR_ Visitor;
    const FParams & Params;
};

class FSVOPathFindingAlgorithmStepper2
{
public:
    virtual ~FSVOPathFindingAlgorithmStepper2() = default;
private:
    virtual bool Step( ENavigationQueryResult::Type & result ) = 0;
};

template< typename _ALGO_ >
class TSVOPathFindingAlgorithmStepper2 : public FSVOPathFindingAlgorithmStepper2
{
public:

    TSVOPathFindingAlgorithmStepper2( const FParams & params ) :
        Algo( params )
    {}

    bool Step( ENavigationQueryResult::Type & result ) override
    {
        return Algo.Step( result );
    }

private:

    _ALGO_ Algo;
};

UCLASS()
class SVONAVIGATION_API USVOPathFindingAlgorithmTest final : public UObject
{
    GENERATED_BODY()

public:
    ENavigationQueryResult::Type GetPath( const FParams & params ) const
    {
        const auto stepper = MakeShared< TSVOPathFindingAlgorithmStepper2< TAlgoAStar< FPathBuilder > > >( params );
    
        int iterations = 0;

        ENavigationQueryResult::Type result = ENavigationQueryResult::Fail;

        while ( stepper->Step( result ) )
        {
            iterations++;
        }

        return result;
    }
    
    template< typename _ALGO_ >
    TSharedPtr< TSVOPathFindingAlgorithmStepper2< _ALGO_ > > GetDebugPathStepper2( const FParams & params )
    {
        return MakeShared< TSVOPathFindingAlgorithmStepper2< TAlgoAStar< FDebugPath > > >( params );
    }
};
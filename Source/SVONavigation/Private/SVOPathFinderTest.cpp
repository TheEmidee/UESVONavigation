#include "SVOPathFinderTest.h"

#include "NavigationSystem.h"
#include "SVONavigationData.h"
#include "SVOPathFinder.h"
#include "SVOPathFindingAlgorithm.h"
#include "SVOPathFindingRenderingComponent.h"

#include <Components/SphereComponent.h>

#if WITH_EDITOR
#include <Engine/Selection.h>
#endif

ASVOPathFinderTest::ASVOPathFinderTest()
{
    PrimaryActorTick.bCanEverTick = false;
    PrimaryActorTick.bStartWithTickEnabled = true;

    SphereComponent = CreateDefaultSubobject< USphereComponent >( TEXT( "SphereComponent" ) );
    RootComponent = SphereComponent;

#if WITH_EDITORONLY_DATA
    RenderingComponent = CreateEditorOnlyDefaultSubobject< USVOPathFindingRenderingComponent >( TEXT( "RenderingComponent" ) );
    if ( RenderingComponent != nullptr )
    {
        RenderingComponent->SetCollisionEnabled( ECollisionEnabled::NoCollision );
    }
#endif

#if WITH_EDITOR
    if ( HasAnyFlags( RF_ClassDefaultObject ) && GetClass() == StaticClass() )
    {
        USelection::SelectObjectEvent.AddStatic( &ASVOPathFinderTest::OnEditorSelectionChanged );
        USelection::SelectionChangedEvent.AddStatic( &ASVOPathFinderTest::OnEditorSelectionChanged );
    }
#endif

    NavAgentProperties = FNavAgentProperties::DefaultProperties;
    AutoStepTimer = 0.2f;
    bAutoComplete = false;
}

#if WITH_EDITOR
void ASVOPathFinderTest::PostEditChangeProperty( FPropertyChangedEvent & property_changed_event )
{
    static const FName NAME_NavigationQueryFilter = GET_MEMBER_NAME_CHECKED( ASVOPathFinderTest, NavigationQueryFilter );

    if ( property_changed_event.Property != nullptr )
    {
        const FName property_name = property_changed_event.MemberProperty->GetFName();
        if ( property_name == NAME_NavigationQueryFilter )
        {
            InitPathFinding();
        }
    }

    Super::PostEditChangeProperty( property_changed_event );
}
#endif

void ASVOPathFinderTest::UpdateDrawing()
{
#if WITH_EDITORONLY_DATA
    if ( HasAnyFlags( RF_ClassDefaultObject ) )
    {
        return;
    }

    if ( RenderingComponent != nullptr && RenderingComponent->GetVisibleFlag() )
    {
        RenderingComponent->MarkRenderStateDirty();

#if WITH_EDITOR
        if ( GEditor != NULL )
        {
            GEditor->RedrawLevelEditingViewports();
        }
#endif // WITH_EDITOR
    }
#endif // WITH_EDITORONLY_DATA
}

void ASVOPathFinderTest::InitPathFinding()
{
    if ( UNavigationSystemV1 * navigation_system = UNavigationSystemV1::GetCurrent( GetWorld() ) )
    {
        if ( auto * navigation_data = navigation_system->GetNavDataForProps( NavAgentProperties ) )
        {
            if ( const auto * svo_navigation_data = Cast< ASVONavigationData >( navigation_data ) )
            {
                if ( OtherActor != nullptr )
                {
                    const auto path_start = GetActorLocation();
                    const auto path_end = OtherActor->GetActorLocation();

                    const FPathFindingQuery Query( this, *svo_navigation_data, path_start, path_end, UNavigationQueryFilter::GetQueryFilter( *svo_navigation_data, this, NavigationQueryFilter ) );
                    Stepper = FSVOPathFinder::GetDebugPathStepper( PathFinderDebugInfos, Query.NavAgentProperties.AgentRadius, *svo_navigation_data, path_start, path_end, Query );

                    PathFinderDebugInfos.Reset();
                    NavigationPath.ResetForRepath();
                    LastStatus = ESVOPathFindingAlgorithmStepperStatus::MustContinue;
                    PathFindingResult = SearchFail;
                    bAutoComplete = false;

                    UpdateDrawing();
                    return;
                }
            }
        }
    }

    ensureAlwaysMsgf( false, TEXT( "Impossible to get the SVO navigation data. Check your NavAgentProperties" ) );
}

void ASVOPathFinderTest::Step()
{
    if ( Stepper.IsValid() && LastStatus != ESVOPathFindingAlgorithmStepperStatus::IsStopped )
    {
        LastStatus = Stepper->Step( PathFindingResult );
        if ( LastStatus == ESVOPathFindingAlgorithmStepperStatus::MustContinue )
        {
            UpdateDrawing();

            if ( bAutoComplete )
            {
                GetWorld()->GetTimerManager().SetTimer( AutoCompleteTimerHandle, this, &ASVOPathFinderTest::Step, AutoStepTimer, false );
                return;
            }
        }
        else if ( PathFindingResult == EGraphAStarResult::SearchSuccess )
        {
            UpdateDrawing();
        }
    }

    bAutoComplete = false;
    GetWorld()->GetTimerManager().ClearAllTimersForObject( this );
}

void ASVOPathFinderTest::AutoCompleteStepByStep()
{
    bAutoComplete = true;
    Step();
}

void ASVOPathFinderTest::AutoCompleteInstantly()
{
    if ( Stepper.IsValid() && LastStatus != ESVOPathFindingAlgorithmStepperStatus::IsStopped )
    {
        do
        {
            LastStatus = Stepper->Step( PathFindingResult );
        } while ( LastStatus == ESVOPathFindingAlgorithmStepperStatus::MustContinue );

        UpdateDrawing();
    }

    GetWorld()->GetTimerManager().ClearAllTimersForObject( this );
}

#if WITH_EDITOR
void ASVOPathFinderTest::OnEditorSelectionChanged( UObject * new_selection )
{
    TArray< ASVOPathFinderTest * > selected_test_actors;
    if ( ASVOPathFinderTest * selected_test_actor = Cast< ASVOPathFinderTest >( new_selection ) )
    {
        selected_test_actors.Add( selected_test_actor );
    }
    else
    {
        if ( USelection * selection = Cast< USelection >( new_selection ) )
        {
            selection->GetSelectedObjects< ASVOPathFinderTest >( selected_test_actors );
        }
    }

    for ( ASVOPathFinderTest * path_finder_test : selected_test_actors )
    {
        /*if ( path_finder_test->QueryTemplate != nullptr && path_finder_test->QueryInstance.IsValid() == false )
        {
            path_finder_test->QueryTemplate->CollectQueryParams( *path_finder_test, path_finder_test->QueryConfig );
            path_finder_test->RunEQSQuery();
        }*/
    }
}
#endif
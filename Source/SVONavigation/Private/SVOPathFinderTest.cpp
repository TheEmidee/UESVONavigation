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
    bUpdatePathAfterMoving = false;
}

#if WITH_EDITOR
void ASVOPathFinderTest::PreEditChange( FProperty * property_about_to_change )
{
    static const FName NAME_OtherActor = GET_MEMBER_NAME_CHECKED( ASVOPathFinderTest, OtherActor );

    if ( property_about_to_change != nullptr && property_about_to_change->GetFName() == NAME_OtherActor && OtherActor != nullptr && OtherActor->OtherActor == this )
    {
        OtherActor->OtherActor = nullptr;
        OtherActor->NavigationPath.ResetForRepath();
        NavigationPath.ResetForRepath();
#if WITH_EDITORONLY_DATA
        OtherActor->RenderingComponent->MarkRenderStateDirty();
        RenderingComponent->MarkRenderStateDirty();
#endif
    }

    Super::PreEditChange( property_about_to_change );
}

void ASVOPathFinderTest::PostEditChangeProperty( FPropertyChangedEvent & property_changed_event )
{
    static const FName NAME_NavigationQueryFilter = GET_MEMBER_NAME_CHECKED( ASVOPathFinderTest, NavigationQueryFilter );
    static const FName NAME_OtherActor = GET_MEMBER_NAME_CHECKED( ASVOPathFinderTest, OtherActor );
    static const FName NAME_UpdatePathAfterMoving = GET_MEMBER_NAME_CHECKED( ASVOPathFinderTest, bUpdatePathAfterMoving );

    if ( property_changed_event.Property != nullptr )
    {
        const FName property_name = property_changed_event.MemberProperty->GetFName();
        if ( property_name == NAME_NavigationQueryFilter )
        {
            InitPathFinding();
        }
        else if ( property_name == NAME_OtherActor )
        {
            if ( OtherActor != nullptr )
            {
                auto * other_actors_old_other_actor = OtherActor->OtherActor;

                OtherActor->OtherActor = this;

#if WITH_EDITORONLY_DATA
                RenderingComponent->MarkRenderStateDirty();
#endif

                if ( other_actors_old_other_actor != nullptr )
                {
                    other_actors_old_other_actor->OtherActor = nullptr;
                    other_actors_old_other_actor->NavigationPath.ResetForRepath();
#if WITH_EDITORONLY_DATA
                    other_actors_old_other_actor->RenderingComponent->MarkRenderStateDirty();
#endif
                }
            }
        }
        else if ( property_name == NAME_UpdatePathAfterMoving )
        {
            if ( bUpdatePathAfterMoving && OtherActor != nullptr )
            {
                OtherActor->bUpdatePathAfterMoving = false;
            }
        }
    }

    Super::PostEditChangeProperty( property_changed_event );
}

void ASVOPathFinderTest::PostEditMove( const bool is_finished )
{
    Super::PostEditMove( is_finished );

    if ( OtherActor != nullptr )
    {
        if ( bUpdatePathAfterMoving )
        {
            InitPathFinding();
            AutoCompleteInstantly();
        }
        else if ( OtherActor->bUpdatePathAfterMoving )
        {
            OtherActor->InitPathFinding();
            OtherActor->AutoCompleteInstantly();
        }
    }
}
#endif

void ASVOPathFinderTest::BeginDestroy()
{
    NavigationPath.ResetForRepath();

    if ( OtherActor != nullptr && OtherActor->OtherActor == this )
    {
        OtherActor->OtherActor = nullptr;
        OtherActor->NavigationPath.ResetForRepath();
    }

    Super::BeginDestroy();
}

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
                    Stepper = FSVOPathFinder::GetDebugPathStepper( PathFinderDebugInfos, Query.NavAgentProperties, *svo_navigation_data, path_start, path_end, Query );

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

void ASVOPathFinderTest::AutoCompleteUntilNextNode()
{
    if ( Stepper.IsValid() && LastStatus != ESVOPathFindingAlgorithmStepperStatus::IsStopped )
    {
        do
        {
            LastStatus = Stepper->Step( PathFindingResult );
        } while ( LastStatus == ESVOPathFindingAlgorithmStepperStatus::MustContinue && Stepper->GetState() != ESVOPathFindingAlgorithmState::ProcessNode );

        UpdateDrawing();
    }

    GetWorld()->GetTimerManager().ClearAllTimersForObject( this );
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

void ASVOPathFinderTest::PauseAutoCompletion()
{
    bAutoComplete = false;
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
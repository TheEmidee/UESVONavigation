#include "SVOPathFinderTest.h"

#include "SVONavigationData.h"
#include "SVOPathFinder.h"
#include "SVOPathFindingAlgorithm.h"

#include <Components/SphereComponent.h>
#include <Engine/Canvas.h>
#include <NavigationSystem.h>

#if WITH_EDITOR
#include <Engine/Selection.h>
#endif

void FSVOPathFindingSceneProxyData::GatherData( const ASVOPathFinderTest & path_finder_test )
{
    StartLocation = path_finder_test.GetStartLocation();
    EndLocation = path_finder_test.GetEndLocation();
    DebugInfos = path_finder_test.GetPathFinderDebugInfos();

    if ( path_finder_test.GetStepperLastStatus() == ESVOPathFindingAlgorithmStepperStatus::IsStopped )
    {
        PathFindingResult = TOptional< EGraphAStarResult >( path_finder_test.GetPathFindingResult() );
    }
    else
    {
        PathFindingResult.Reset();
    }
}

FSVOPathFindingSceneProxy::FSVOPathFindingSceneProxy( const UPrimitiveComponent & component, const FSVOPathFindingSceneProxyData & proxy_data ) :
    FDebugRenderSceneProxy( &component )
{
    DrawType = SolidAndWireMeshes;
    TextWithoutShadowDistance = 1500;
    bWantsSelectionOutline = false;
    ViewFlagName = TEXT( "Navigation" );
    ViewFlagIndex = static_cast< uint32 >( FEngineShowFlags::FindIndexByName( *ViewFlagName ) );

    RenderingComponent = MakeWeakObjectPtr( const_cast< USVOPathFindingRenderingComponent * >( Cast< USVOPathFindingRenderingComponent >( &component ) ) );
    PathFinderTest = RenderingComponent->GetPathFinderTest();
    DebugDrawOptions = PathFinderTest->GetDebugDrawOptions();

    const auto add_text = [ texts = &Texts ]( const FSVOPathFinderDebugNodeCost & debug_node_cost ) {
        texts->Emplace( FText3d( FString::SanitizeFloat( debug_node_cost.Cost ), FVector( 0.0f, 0.0f, 50.0f ) + ( debug_node_cost.From.Location + debug_node_cost.To.Location ) / 2.0f, FLinearColor::White ) );
    };

    if ( DebugDrawOptions.bDrawLastProcessedNode )
    {
        Lines.Emplace( FDebugLine( proxy_data.DebugInfos.LastLastProcessedSingleNode.From.Location, proxy_data.DebugInfos.LastLastProcessedSingleNode.To.Location, FColor::Blue, 2.0f ) );

        if ( DebugDrawOptions.bDrawLastProcessedNodeCost )
        {
            add_text( proxy_data.DebugInfos.LastLastProcessedSingleNode );
        }
    }

    if ( DebugDrawOptions.bDrawLastProcessedNeighbors )
    {
        for ( const auto & neighbor : proxy_data.DebugInfos.ProcessedNeighbors )
        {
            Lines.Emplace( FDebugLine( neighbor.From.Location, neighbor.To.Location, neighbor.bIsClosed ? FColor::Orange : FColor::Green, 1.0f ) );

            if ( DebugDrawOptions.bDrawNeighborsCost )
            {
                add_text( neighbor );
            }
        }
    }

    if ( proxy_data.PathFindingResult.Get( EGraphAStarResult::SearchFail ) == EGraphAStarResult::SearchSuccess 
        || DebugDrawOptions.bDrawBestPath )
    {
        const auto & best_path_points = proxy_data.DebugInfos.CurrentBestPath.GetPathPoints();

        ArrowHeadLocations.Reserve( best_path_points.Num() );

        for ( auto index = 0; index < best_path_points.Num() - 1; index++ )
        {
            const auto from = best_path_points[ index ];
            const auto to = best_path_points[ index + 1 ];

            Lines.Emplace( from, to, FColor::Red, 3.0f );
            Boxes.Emplace( FBox::BuildAABB( from, FVector( 20.0f ) ), FColor::Red );
            ArrowHeadLocations.Emplace( from, to );
        }
    }

    ActorOwner = component.GetOwner();
}

SIZE_T FSVOPathFindingSceneProxy::GetTypeHash() const
{
    static size_t UniquePointer;
    return reinterpret_cast< size_t >( &UniquePointer );
}

FPrimitiveViewRelevance FSVOPathFindingSceneProxy::GetViewRelevance( const FSceneView * view ) const
{
    FPrimitiveViewRelevance Result;
    Result.bDrawRelevance = /*view->Family->EngineShowFlags.GetSingleFlag(ViewFlagIndex) &&*/ IsShown( view ) && ( !DebugDrawOptions.bDrawOnlyWhenSelected || SafeIsActorSelected() );
    Result.bDynamicRelevance = true;
    // ideally the TranslucencyRelevance should be filled out by the material, here we do it conservative
    Result.bSeparateTranslucency = Result.bNormalTranslucency = IsShown( view );
    return Result;
}

void FSVOPathFindingSceneProxy::GetDynamicMeshElements( const TArray< const FSceneView * > & views, const FSceneViewFamily & view_family, const uint32 visibility_map, FMeshElementCollector & collector ) const
{
    FDebugRenderSceneProxy::GetDynamicMeshElements( views, view_family, visibility_map, collector );

    for ( int32 view_index = 0; view_index < views.Num(); view_index++ )
    {
        const FSceneView * view = views[ view_index ];
        FPrimitiveDrawInterface * pdi = collector.GetPDI( view_index );

        if ( visibility_map & ( 1 << view_index ) )
        {
            for ( const auto & pair : ArrowHeadLocations )
            {
                DrawArrowHead( pdi, pair.Value, pair.Key, 50.f, FColor::Red, SDPG_World, 10.0f );
            }
        }
    }
}

bool FSVOPathFindingSceneProxy::SafeIsActorSelected() const
{
    if ( ActorOwner )
    {
        return ActorOwner->IsSelected();
    }

    return false;
}

USVOPathFindingRenderingComponent::USVOPathFindingRenderingComponent()
{
}

FPrimitiveSceneProxy * USVOPathFindingRenderingComponent::CreateSceneProxy()
{
    FSVOPathFindingSceneProxyData proxy_data;
    GatherData( proxy_data, *GetPathFinderTest() );

    if ( FSVOPathFindingSceneProxy * new_scene_proxy = new FSVOPathFindingSceneProxy( *this, proxy_data ) )
    {
        if ( IsInGameThread() )
        {
            RenderingDebugDrawDelegateHelper.InitDelegateHelper( *new_scene_proxy );
            RenderingDebugDrawDelegateHelper.ReregisterDebugDrawDelgate();
        }

        return new_scene_proxy;
    }

    return nullptr;
}

void USVOPathFindingRenderingComponent::CreateRenderState_Concurrent( FRegisterComponentContext * Context )
{
    Super::CreateRenderState_Concurrent( Context );

    RenderingDebugDrawDelegateHelper.RegisterDebugDrawDelgate();
}

void USVOPathFindingRenderingComponent::DestroyRenderState_Concurrent()
{
    RenderingDebugDrawDelegateHelper.UnregisterDebugDrawDelgate();

    Super::DestroyRenderState_Concurrent();
}

FBoxSphereBounds USVOPathFindingRenderingComponent::CalcBounds( const FTransform & local_to_world ) const
{
    FBoxSphereBounds result;

    if ( auto * owner = Cast< ASVOPathFinderTest >( GetOwner() ) )
    {
        FVector center, extent;
        owner->GetActorBounds( false, center, extent );
        result = FBoxSphereBounds( FBox::BuildAABB( center, extent ) );
    }

    return result;
}

void USVOPathFindingRenderingComponent::GatherData( FSVOPathFindingSceneProxyData & proxy_data, const ASVOPathFinderTest & path_finder_test )
{
    proxy_data.GatherData( path_finder_test );
}

void FSVOPathFindingRenderingDebugDrawDelegateHelper::InitDelegateHelper( const FSVOPathFindingSceneProxy & InSceneProxy )
{
    Super::InitDelegateHelper( &InSceneProxy );

    ActorOwner = InSceneProxy.ActorOwner;
    //QueryDataSource = InSceneProxy->QueryDataSource;
    DebugDrawOptions = InSceneProxy.DebugDrawOptions;
}

void FSVOPathFindingRenderingDebugDrawDelegateHelper::DrawDebugLabels( UCanvas * Canvas, APlayerController * PC )
{
    if ( !ActorOwner || ( ActorOwner->IsSelected() == false && DebugDrawOptions.bDrawOnlyWhenSelected == true ) )
    {
        return;
    }

    // little hacky test but it's the only way to remove text rendering from bad worlds, when using UDebugDrawService for it
    if ( Canvas && Canvas->SceneView && Canvas->SceneView->Family && Canvas->SceneView->Family->Scene && Canvas->SceneView->Family->Scene->GetWorld() != ActorOwner->GetWorld() )
    {
        return;
    }

    FDebugDrawDelegateHelper::DrawDebugLabels( Canvas, PC );
}

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
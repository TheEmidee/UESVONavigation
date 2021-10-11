#include "SVOPathFinderTest.h"

#include "NavigationSystem.h"
#include "SVONavigationData.h"
#include "SVOPathFinder.h"
#include "SVOPathFindingAlgorithm.h"
#include "SVOPathFindingRenderingComponent.h"

#include <Components/BillboardComponent.h>
#include <UObject/ConstructorHelpers.h>

#if WITH_EDITOR
#include <Engine/Selection.h>
#endif

ASVOPathFinderTest::ASVOPathFinderTest()
{
    PrimaryActorTick.bCanEverTick = false;
    PrimaryActorTick.bStartWithTickEnabled = true;

    StartLocationComponent = CreateDefaultSubobject< UBillboardComponent >( TEXT( "StartLocationComponent" ) );
    RootComponent = StartLocationComponent;

    struct FConstructorStatics_StartObject
    {
        ConstructorHelpers::FObjectFinderOptional< UTexture2D > TextureObject;
        FName ID_Misc;
        FText NAME_Misc;
        FConstructorStatics_StartObject() :
            TextureObject( TEXT( "/Engine/EditorResources/Spawn_Point" ) ),
            ID_Misc( TEXT( "Misc" ) ),
            NAME_Misc( NSLOCTEXT( "SpriteCategory", "Misc", "Misc" ) )
        {
        }
    };
    static FConstructorStatics_StartObject ConstructorStatics_Start;

    StartLocationComponent->Sprite = ConstructorStatics_Start.TextureObject.Get();
    StartLocationComponent->SetRelativeScale3D( FVector( 1, 1, 1 ) );
    StartLocationComponent->bHiddenInGame = true;

    //SpriteComponent->Mobility = EComponentMobility::Static;
#if WITH_EDITORONLY_DATA
    StartLocationComponent->SpriteInfo.Category = ConstructorStatics_Start.ID_Misc;
    StartLocationComponent->SpriteInfo.DisplayName = ConstructorStatics_Start.NAME_Misc;
#endif
    
    StartLocationComponent->bIsScreenSizeScaled = true;

    struct FConstructorStatics_TargetObject
    {
        ConstructorHelpers::FObjectFinderOptional< UTexture2D > TextureObject;
        FName ID_Misc;
        FText NAME_Misc;
        FConstructorStatics_TargetObject() :
            TextureObject( TEXT( "/Engine/EditorResources/Waypoint" ) ),
            ID_Misc( TEXT( "Misc" ) ),
            NAME_Misc( NSLOCTEXT( "SpriteCategory", "Misc", "Misc" ) )
        {
        }
    };
    static FConstructorStatics_TargetObject ConstructorStatics_Target;

    EndLocationComponent = CreateDefaultSubobject< UBillboardComponent >( TEXT( "EndLocationComponent" ) );
    EndLocationComponent->SetupAttachment( RootComponent );

    EndLocationComponent->Sprite = ConstructorStatics_Target.TextureObject.Get();
    EndLocationComponent->SetRelativeScale3D( FVector( 1, 1, 1 ) );
    EndLocationComponent->bHiddenInGame = true;
    //SpriteComponent->Mobility = EComponentMobility::Static;
    
#if WITH_EDITORONLY_DATA
    EndLocationComponent->SpriteInfo.Category = ConstructorStatics_Target.ID_Misc;
    EndLocationComponent->SpriteInfo.DisplayName = ConstructorStatics_Target.NAME_Misc;
#endif
    
    EndLocationComponent->bIsScreenSizeScaled = true;

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
                const auto path_start = StartLocationComponent->GetComponentLocation();
                const auto path_end = EndLocationComponent->GetComponentLocation();

                const FPathFindingQuery Query( this, *svo_navigation_data, path_start, path_end, UNavigationQueryFilter::GetQueryFilter( *svo_navigation_data, this, NavigationQueryFilter ) );
                Stepper = FSVOPathFinder::GetDebugPathStepper( PathFinderDebugInfos, *svo_navigation_data, StartLocationComponent->GetComponentLocation(), EndLocationComponent->GetComponentLocation(), Query );
                
                NavigationPath.ResetForRepath();
                bFoundPath = false;

                UpdateDrawing();
                return;
            }
        }
    }

    ensureAlwaysMsgf( false, TEXT( "Impossible to get the SVO navigation data. Check your NavAgentProperties" ) );
}

void ASVOPathFinderTest::Step()
{
    if ( Stepper.IsValid() && !bFoundPath )
    {
        ENavigationQueryResult::Type result = ENavigationQueryResult::Fail;
        if ( Stepper->Step( result ) )
        {
            UpdateDrawing();
        
            if ( bAutoComplete )
            {
                GetWorld()->GetTimerManager().SetTimer( AutoCompleteTimerHandle, this, &ASVOPathFinderTest::Step, AutoStepTimer, false );
                return;
            }
        }
        else if ( result == ENavigationQueryResult::Success )
        {
            bFoundPath = true;
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
    if ( Stepper.IsValid() && !bFoundPath )
    {
        ENavigationQueryResult::Type result = ENavigationQueryResult::Fail;

        while ( Stepper->Step( result ) )
        {
        }

        UpdateDrawing();
        
        if ( result == ENavigationQueryResult::Success )
        {
            bFoundPath = true;
        }
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
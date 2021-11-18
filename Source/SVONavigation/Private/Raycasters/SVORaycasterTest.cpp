#include "Raycasters/SVORaycasterTest.h"

#include "Raycasters/SVORaycaster_OctreeTraversal.h"
#include "SVONavigationData.h"

#include <Components/SphereComponent.h>
#include <NavigationSystem.h>

void FSVORayCasterSceneProxyData::GatherData( const ASVORaycasterTest & ray_caster_test )
{
}

FSVORayCasterSceneProxy::FSVORayCasterSceneProxy( const UPrimitiveComponent & component, const FSVORayCasterSceneProxyData & proxy_data ) :
    FDebugRenderSceneProxy( &component )
{
}

SIZE_T FSVORayCasterSceneProxy::GetTypeHash() const
{
    static size_t UniquePointer;
    return reinterpret_cast< size_t >( &UniquePointer );
}

FPrimitiveViewRelevance FSVORayCasterSceneProxy::GetViewRelevance( const FSceneView * view ) const
{
    FPrimitiveViewRelevance Result;
    Result.bDrawRelevance = /*view->Family->EngineShowFlags.GetSingleFlag(ViewFlagIndex) &&*/ IsShown( view ) /* && ( !DebugDrawOptions.bDrawOnlyWhenSelected || SafeIsActorSelected() ) */;
    Result.bDynamicRelevance = true;
    // ideally the TranslucencyRelevance should be filled out by the material, here we do it conservative
    Result.bSeparateTranslucency = Result.bNormalTranslucency = IsShown( view );
    return Result;
}

bool FSVORayCasterSceneProxy::SafeIsActorSelected() const
{
    if ( ActorOwner )
    {
        return ActorOwner->IsSelected();
    }

    return false;
}

ASVORaycasterTest * USVORayCasterRenderingComponent::GetRayCasterTest() const
{
    return Cast< ASVORaycasterTest >( GetOwner() );
}

FPrimitiveSceneProxy * USVORayCasterRenderingComponent::CreateSceneProxy()
{
    FSVORayCasterSceneProxyData proxy_data;
    proxy_data.GatherData( *GetRayCasterTest() );

    if ( FSVORayCasterSceneProxy * new_scene_proxy = new FSVORayCasterSceneProxy( *this, proxy_data ) )
    {
        return new_scene_proxy;
    }

    return nullptr;
}

FBoxSphereBounds USVORayCasterRenderingComponent::CalcBounds( const FTransform & local_to_world ) const
{
    FBoxSphereBounds result;

    if ( const auto * owner = GetRayCasterTest() )
    {
        FVector center, extent;
        owner->GetActorBounds( false, center, extent );
        result = FBoxSphereBounds( FBox::BuildAABB( center, extent ) );
    }

    return result;
}

ASVORaycasterTest::ASVORaycasterTest()
{
    SphereComponent = CreateDefaultSubobject< USphereComponent >( TEXT( "SphereComponent" ) );
    SphereComponent->InitSphereRadius( 100.0f );
    RootComponent = SphereComponent;

#if WITH_EDITORONLY_DATA
    RenderingComponent = CreateEditorOnlyDefaultSubobject< USVORayCasterRenderingComponent >( TEXT( "RenderingComponent" ) );
    if ( RenderingComponent != nullptr )
    {
        RenderingComponent->SetCollisionEnabled( ECollisionEnabled::NoCollision );
    }
#endif

    PrimaryActorTick.bCanEverTick = false;

    Raycaster = NewObject< USVORayCaster_OctreeTraversal >();
    NavAgentProperties.PreferredNavData = ASVONavigationData::StaticClass();
    NavAgentProperties.AgentRadius = 100.0f;
    bUpdatePathAfterMoving = true;
}

#if WITH_EDITOR
void ASVORaycasterTest::PreEditChange( FProperty * property_about_to_change )
{
    static const FName NAME_OtherActor = GET_MEMBER_NAME_CHECKED( ASVORaycasterTest, OtherActor );

    if ( property_about_to_change != nullptr && property_about_to_change->GetFName() == NAME_OtherActor && OtherActor != nullptr && OtherActor->OtherActor == this )
    {
        OtherActor->OtherActor = nullptr;
#if WITH_EDITORONLY_DATA
        OtherActor->RenderingComponent->MarkRenderStateDirty();
        RenderingComponent->MarkRenderStateDirty();
#endif
    }

    Super::PreEditChange( property_about_to_change );
}

void ASVORaycasterTest::PostEditChangeProperty( FPropertyChangedEvent & property_changed_event )
{
    static const FName NAME_OtherActor = GET_MEMBER_NAME_CHECKED( ASVORaycasterTest, OtherActor );
    static const FName NAME_UpdatePathAfterMoving = GET_MEMBER_NAME_CHECKED( ASVORaycasterTest, bUpdatePathAfterMoving );

    if ( property_changed_event.Property != nullptr )
    {
        const FName property_name = property_changed_event.MemberProperty->GetFName();
        if ( property_name == NAME_OtherActor )
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

void ASVORaycasterTest::PostEditMove( const bool is_finished )
{
    Super::PostEditMove( is_finished );

    if ( OtherActor != nullptr )
    {
        if ( bUpdatePathAfterMoving )
        {
            DoRaycast();
        }
        else if ( OtherActor->bUpdatePathAfterMoving )
        {
            OtherActor->DoRaycast();
        }
    }
}
#endif

void ASVORaycasterTest::BeginDestroy()
{
    if ( OtherActor != nullptr && OtherActor->OtherActor == this )
    {
        OtherActor->OtherActor = nullptr;
    }

    Super::BeginDestroy();
}

void ASVORaycasterTest::UpdateDrawing()
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

void ASVORaycasterTest::DoRaycast()
{
    if ( OtherActor == nullptr )
    {
        return;
    }

    if ( Raycaster == nullptr )
    {
        return;
    }

    const auto has_los = Raycaster->HasLineOfSight( this, GetActorLocation(), OtherActor->GetActorLocation(), NavAgentProperties );
}
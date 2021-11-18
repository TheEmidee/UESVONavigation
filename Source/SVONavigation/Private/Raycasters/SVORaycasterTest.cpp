#include "Raycasters/SVORaycasterTest.h"

#include "Raycasters/SVORayCaster.h"
#include "Raycasters/SVORaycaster_OctreeTraversal.h"
#include "SVONavigationData.h"

#include <Components/SphereComponent.h>
#include <NavigationSystem.h>

void FSVORayCasterSceneProxyData::GatherData( const ASVORaycasterTest & ray_caster_test )
{
    DebugInfos = ray_caster_test.GetDebugInfos();
}

FSVORayCasterSceneProxy::FSVORayCasterSceneProxy( const UPrimitiveComponent & component, const FSVORayCasterSceneProxyData & proxy_data ) :
    FDebugRenderSceneProxy( &component )
{
    DrawType = EDrawType::SolidAndWireMeshes;
    RayCasterTest = Cast< ASVORaycasterTest >( component.GetOwner() );

    const auto & debug_draw_options = RayCasterTest->GetDebugDrawOptions();

    if ( proxy_data.DebugInfos.NavigationData == nullptr )
    {
        return;
    }

    if ( !debug_draw_options.bEnableDebugDraw )
    {
        return;
    }

    Lines.Emplace( proxy_data.DebugInfos.RayCastStartLocation, proxy_data.DebugInfos.RayCastEndLocation, FColor::Magenta, 5.0f );

    const auto layer_count = proxy_data.DebugInfos.NavigationData->GetData().GetLayerCount();
    const auto corrected_layer_index = FMath::Clamp( static_cast< int >( debug_draw_options.LayerIndexToDraw ), 0, layer_count - 1 );

    if ( debug_draw_options.bDrawLayerNodes )
    {
        if ( corrected_layer_index == 0 )
        {
            const auto & layer_zero = proxy_data.DebugInfos.NavigationData->GetData().GetLayer( 0 );
            const auto & layer_zero_nodes = layer_zero.GetNodes();

            for ( const auto & traversed_leaf_node : proxy_data.DebugInfos.TraversedLeafNodes )
            {
                if ( const auto * node_ptr = layer_zero_nodes.FindByPredicate( [ node_address = traversed_leaf_node.NodeAddress ]( const FSVONode & layer_zero_node ) {
                         return layer_zero_node.FirstChild == node_address;
                     } ) )
                {
                    const FSVONodeAddress node_address( 0, node_ptr->MortonCode );

                    const auto node_position = proxy_data.DebugInfos.NavigationData->GetNodePositionFromAddress( node_address );
                    const auto node_half_extent = proxy_data.DebugInfos.NavigationData->GetData().GetLayer( node_address.LayerIndex ).GetVoxelHalfExtent();

                    Boxes.Emplace( FBox::BuildAABB( node_position, FVector( node_half_extent ) ), traversed_leaf_node.bIsOccluded ? FColor::Orange : FColor::Green );
                }
            }
        }
        else
        {
            for ( const auto & traversed_node : proxy_data.DebugInfos.TraversedNodes )
            {
                if ( traversed_node.NodeAddress.LayerIndex != corrected_layer_index )
                {
                    continue;
                }

                const auto node_position = proxy_data.DebugInfos.NavigationData->GetNodePositionFromAddress( traversed_node.NodeAddress );
                const auto node_half_extent = proxy_data.DebugInfos.NavigationData->GetData().GetLayer( traversed_node.NodeAddress.LayerIndex ).GetVoxelHalfExtent();

                Boxes.Emplace( FBox::BuildAABB( node_position, FVector( node_half_extent ) ), traversed_node.bIsOccluded ? FColor::Orange : FColor::Green );
            }
        }
    }
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
    const FBoxSphereBounds result;

    if ( const auto * owner = GetRayCasterTest() )
    {
        return owner->GetBoundingBoxContainingOtherActorAndMe();
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

FBoxSphereBounds ASVORaycasterTest::GetBoundingBoxContainingOtherActorAndMe() const
{
    TArray< FVector > points;
    points.Reserve( 2 );

    points.Add( GetActorLocation() );

    if ( OtherActor != nullptr )
    {
        points.Add( OtherActor->GetActorLocation() );
    }

    return FBoxSphereBounds( FBox( points ) );
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

    Raycaster->SetObserver( MakeShared< FSVORayCasterObserver_GenerateDebugInfos >( RayCasterDebugInfos ) );
    Raycaster->HasLineOfSight( this, GetActorLocation(), OtherActor->GetActorLocation(), NavAgentProperties );

    UpdateDrawing();
}
#include "SVONavDataRenderingComponent.h"

#include "SVONavigationData.h"

#include <Engine/CollisionProfile.h>

namespace
{
    void DrawDebugBox( FPrimitiveDrawInterface * PDI, FVector const & Center, FVector const & Box, FColor const & Color )
    {
        // no debug line drawing on dedicated server
        if ( PDI != nullptr )
        {
            PDI->DrawLine( Center + FVector( Box.X, Box.Y, Box.Z ), Center + FVector( Box.X, -Box.Y, Box.Z ), Color, SDPG_World, 5.0f );
            PDI->DrawLine( Center + FVector( Box.X, -Box.Y, Box.Z ), Center + FVector( -Box.X, -Box.Y, Box.Z ), Color, SDPG_World, 5.0f );
            PDI->DrawLine( Center + FVector( -Box.X, -Box.Y, Box.Z ), Center + FVector( -Box.X, Box.Y, Box.Z ), Color, SDPG_World, 5.0f );
            PDI->DrawLine( Center + FVector( -Box.X, Box.Y, Box.Z ), Center + FVector( Box.X, Box.Y, Box.Z ), Color, SDPG_World, 5.0f );

            PDI->DrawLine( Center + FVector( Box.X, Box.Y, -Box.Z ), Center + FVector( Box.X, -Box.Y, -Box.Z ), Color, SDPG_World, 5.0f );
            PDI->DrawLine( Center + FVector( Box.X, -Box.Y, -Box.Z ), Center + FVector( -Box.X, -Box.Y, -Box.Z ), Color, SDPG_World, 5.0f );
            PDI->DrawLine( Center + FVector( -Box.X, -Box.Y, -Box.Z ), Center + FVector( -Box.X, Box.Y, -Box.Z ), Color, SDPG_World, 5.0f );
            PDI->DrawLine( Center + FVector( -Box.X, Box.Y, -Box.Z ), Center + FVector( Box.X, Box.Y, -Box.Z ), Color, SDPG_World, 5.0f );

            PDI->DrawLine( Center + FVector( Box.X, Box.Y, Box.Z ), Center + FVector( Box.X, Box.Y, -Box.Z ), Color, SDPG_World, 5.0f );
            PDI->DrawLine( Center + FVector( Box.X, -Box.Y, Box.Z ), Center + FVector( Box.X, -Box.Y, -Box.Z ), Color, SDPG_World, 5.0f );
            PDI->DrawLine( Center + FVector( -Box.X, -Box.Y, Box.Z ), Center + FVector( -Box.X, -Box.Y, -Box.Z ), Color, SDPG_World, 5.0f );
            PDI->DrawLine( Center + FVector( -Box.X, Box.Y, Box.Z ), Center + FVector( -Box.X, Box.Y, -Box.Z ), Color, SDPG_World, 5.0f );
        }
    }
}

void FSVONavigationSceneProxyData::Reset()
{
    OctreeBounds.Reset();
}

void FSVONavigationSceneProxyData::Serialize( FArchive & Ar )
{
    int32 NumBounds = OctreeBounds.Num();
    Ar << NumBounds;
    if ( Ar.IsLoading() )
    {
        OctreeBounds.SetNum( NumBounds );
    }

    for ( int32 Idx = 0; Idx < NumBounds; Idx++ )
    {
        Ar << OctreeBounds[ Idx ].Center;
        Ar << OctreeBounds[ Idx ].Extent;
    }
}

uint32 FSVONavigationSceneProxyData::GetAllocatedSize() const
{
    return OctreeBounds.GetAllocatedSize();
}

void FSVONavigationSceneProxyData::GatherData( const ASVONavigationData * navigation_data )
{
    Reset();

    const auto & navigation_bounds = navigation_data->GetNavigationBoundsData();

    for ( const auto & bounds : navigation_bounds )
    {
        OctreeBounds.Emplace_GetRef( FBoxCenterAndExtent( bounds.Value.GetBox() ) );
    }
}

SIZE_T FSVONavigationMeshSceneProxy::GetTypeHash() const
{
    static size_t UniquePointer;
    return reinterpret_cast< size_t >( &UniquePointer );
}

FSVONavigationMeshSceneProxy::FSVONavigationMeshSceneProxy( const UPrimitiveComponent * InComponent, FSVONavigationSceneProxyData * InProxyData ) :
    FDebugRenderSceneProxy( InComponent )
{
    DrawType = EDrawType::SolidAndWireMeshes;

    if ( InProxyData )
    {
        ProxyData = *InProxyData;
        //Boxes.Append( InProxyData->AuxBoxes );
    }

    RenderingComponent = MakeWeakObjectPtr( const_cast< USVONavDataRenderingComponent * >( Cast< USVONavDataRenderingComponent >( InComponent ) ) );
}

FSVONavigationMeshSceneProxy::~FSVONavigationMeshSceneProxy()
{
}

void FSVONavigationMeshSceneProxy::GetDynamicMeshElements( const TArray< const FSceneView * > & Views, const FSceneViewFamily & ViewFamily, uint32 VisibilityMap, FMeshElementCollector & Collector ) const
{
    FDebugRenderSceneProxy::GetDynamicMeshElements( Views, ViewFamily, VisibilityMap, Collector );

    for ( int32 ViewIndex = 0; ViewIndex < Views.Num(); ViewIndex++ )
    {
        if ( VisibilityMap & ( 1 << ViewIndex ) )
        {
            const FSceneView * View = Views[ ViewIndex ];
            const bool bVisible = !!View->Family->EngineShowFlags.Navigation;
            if ( !bVisible )
            {
                continue;
            }
            FPrimitiveDrawInterface * PDI = Collector.GetPDI( ViewIndex );

            const auto & octree_bounds = ProxyData.GetOctreeBounds();

            for ( int32 Index = 0; Index < octree_bounds.Num(); ++Index )
            {
                const FBoxCenterAndExtent & ProxyBounds = octree_bounds[ Index ];
                DrawDebugBox( PDI, ProxyBounds.Center, ProxyBounds.Extent, FColor::White );
            }
        }
    }
}

FPrimitiveViewRelevance FSVONavigationMeshSceneProxy::GetViewRelevance( const FSceneView * View ) const
{
    const bool bVisible = !!View->Family->EngineShowFlags.Navigation;
    FPrimitiveViewRelevance Result;
    Result.bDrawRelevance = bVisible && IsShown( View );
    Result.bDynamicRelevance = true;
    // ideally the TranslucencyRelevance should be filled out by the material, here we do it conservative
    Result.bSeparateTranslucency = Result.bNormalTranslucency = bVisible && IsShown( View );
    return Result;
}

USVONavDataRenderingComponent::USVONavDataRenderingComponent()
{
    SetCollisionProfileName( UCollisionProfile::NoCollision_ProfileName );

    bIsEditorOnly = true;
    bSelectable = false;
}

FPrimitiveSceneProxy * USVONavDataRenderingComponent::CreateSceneProxy()
{
#if !UE_BUILD_SHIPPING && !UE_BUILD_TEST
    FSVONavigationMeshSceneProxy * proxy = nullptr;

    if ( IsVisible() )
    {
        if ( const ASVONavigationData * navigation_data = Cast< ASVONavigationData >( GetOwner() ) )
        {
            if ( navigation_data->HasDebugDrawingEnabled() )
            {
                FSVONavigationSceneProxyData ProxyData;
                GatherData( *navigation_data, ProxyData );

                proxy = new FSVONavigationMeshSceneProxy( this, &ProxyData );
            }
        }
    }

    return proxy;
#else
    return nullptr;
#endif
}

void USVONavDataRenderingComponent::GatherData( const ASVONavigationData & navigation_data, FSVONavigationSceneProxyData & OutProxyData ) const
{
    OutProxyData.GatherData( &navigation_data );
}

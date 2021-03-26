#include "SVONavigationVolumeProperties.h"

#include "SVONavigationSystem.h"
#include "SVONavigationVolume.h"

#include <DetailCategoryBuilder.h>
#include <DetailCustomizations/Private/BrushDetails.h>
#include <DetailLayoutBuilder.h>
#include <DetailWidgetRow.h>

#define LOCTEXT_NAMESPACE "SVONavigationVolumeProperties"

TSharedRef< IDetailCustomization > FSVONavigationVolumeProperties::MakeInstance()
{
    return MakeShareable( new FSVONavigationVolumeProperties );
}

void FSVONavigationVolumeProperties::CustomizeDetails( IDetailLayoutBuilder & DetailBuilder )
{
    TSharedPtr< IPropertyHandle > PrimaryTickProperty = DetailBuilder.GetProperty( GET_MEMBER_NAME_CHECKED( UActorComponent, PrimaryComponentTick ) );

    if ( PrimaryTickProperty->IsValidHandle() && DetailBuilder.HasClassDefaultObject() )
    {
        IDetailCategoryBuilder & TickCategory = DetailBuilder.EditCategory( "ComponentTick" );
        TickCategory.AddProperty( PrimaryTickProperty->GetChildHandle( GET_MEMBER_NAME_CHECKED( FTickFunction, bStartWithTickEnabled ) ) );
        TickCategory.AddProperty( PrimaryTickProperty->GetChildHandle( GET_MEMBER_NAME_CHECKED( FTickFunction, TickInterval ) ) );
        TickCategory.AddProperty( PrimaryTickProperty->GetChildHandle( GET_MEMBER_NAME_CHECKED( FTickFunction, bTickEvenWhenPaused ) ), EPropertyLocation::Advanced );
        TickCategory.AddProperty( PrimaryTickProperty->GetChildHandle( GET_MEMBER_NAME_CHECKED( FTickFunction, bAllowTickOnDedicatedServer ) ), EPropertyLocation::Advanced );
        TickCategory.AddProperty( PrimaryTickProperty->GetChildHandle( GET_MEMBER_NAME_CHECKED( FTickFunction, TickGroup ) ), EPropertyLocation::Advanced );
    }

    PrimaryTickProperty->MarkHiddenByCustomization();
    DetailBuilder.HideCategory( "BrushSettings" );
    DetailBuilder.HideCategory( "Navigation" );
    DetailBuilder.HideCategory( "Tags" );
    DetailBuilder.HideCategory( "Collision" );
    DetailBuilder.HideCategory( "HLOD" );
    DetailBuilder.HideCategory( "Mobile" );
    DetailBuilder.HideCategory( "Actor" );

     const TArray<TWeakObjectPtr<UObject>> &SelectedObjects = DetailBuilder.GetSelectedObjects();
     for (int32 ObjectIndex = 0; ObjectIndex < SelectedObjects.Num(); ++ObjectIndex)
     {
     	const TWeakObjectPtr<UObject>& CurrentObject = SelectedObjects[ObjectIndex];
     	if (CurrentObject.IsValid())
     	{
     		ASVONavigationVolume* CurrentVolume = Cast<ASVONavigationVolume>(CurrentObject.Get());
     		if (CurrentVolume != nullptr)
     		{
     			Volume = CurrentVolume;
     			break;
     		}
     	}
     }

    DetailBuilder.EditCategory( "SVONavigation" )
        .AddCustomRow( NSLOCTEXT( "SVONavigationVolume", "Build Octree", "Build Octree" ) )
        .ValueContent()
        .MaxDesiredWidth( 125.f )
        .MinDesiredWidth( 125.f )
            [ SNew( SButton )
                    .ContentPadding( 2 )
                    .VAlign( VAlign_Center )
                    .HAlign( HAlign_Center )
                    .OnClicked( this, &FSVONavigationVolumeProperties::OnBuildOctree )
                        [ SNew( STextBlock )
                                .Font( IDetailLayoutBuilder::GetDetailFont() )
                                .Text( NSLOCTEXT( "SVONavigationVolume", "Build Octree", "Build Octree" ) ) ] ];

    DetailBuilder.EditCategory( "SVONavigation" )
        .AddCustomRow( NSLOCTEXT( "SVONavigationVolume", "Clear Octree", "Clear Octree" ) )
        .ValueContent()
        .MaxDesiredWidth( 125.f )
        .MinDesiredWidth( 125.f )
            [ SNew( SButton )
                    .ContentPadding( 2 )
                    .VAlign( VAlign_Center )
                    .HAlign( HAlign_Center )
                    .OnClicked( this, &FSVONavigationVolumeProperties::OnClearOctree )
                        [ SNew( STextBlock )
                                .Font( IDetailLayoutBuilder::GetDetailFont() )
                                .Text( NSLOCTEXT( "SVONavigationVolume", "Clear Octree", "Clear Octree" ) ) ] ];
}

FReply FSVONavigationVolumeProperties::OnBuildOctree() const
{
    if ( auto * navigation_system = GEngine->GetEngineSubsystem< USVONavigationSystem >() )
    {
        //navigation_system->GenerateSVO( *Volume );
    }
    return FReply::Handled();
}

FReply FSVONavigationVolumeProperties::OnClearOctree() const
{
    if ( auto * navigation_system = GEngine->GetEngineSubsystem< USVONavigationSystem >() )
    {
    }

    return FReply::Handled();
}

#undef LOCTEXT_NAMESPACE

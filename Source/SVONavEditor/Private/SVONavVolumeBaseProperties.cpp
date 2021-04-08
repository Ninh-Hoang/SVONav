#include "SVONavVolumeBaseProperties.h"
#include "SVONavVolumeBase.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "DetailCategoryBuilder.h"
#include "DetailCustomizations/Private/BrushDetails.h"

#define LOCTEXT_NAMESPACE "NavVolumeProperties"

TSharedRef<IDetailCustomization> FSVONavVolumeBaseProperties::MakeInstance()
{
	return MakeShareable( new FSVONavVolumeBaseProperties);
}

void FSVONavVolumeBaseProperties::CustomizeDetails(IDetailLayoutBuilder& DetailBuilder)
{
	TSharedPtr<IPropertyHandle> PrimaryTickProperty = DetailBuilder.GetProperty(GET_MEMBER_NAME_CHECKED(UActorComponent, PrimaryComponentTick));

	if (PrimaryTickProperty->IsValidHandle() && DetailBuilder.HasClassDefaultObject())
	{
		IDetailCategoryBuilder& TickCategory = DetailBuilder.EditCategory("ComponentTick");
		TickCategory.AddProperty(PrimaryTickProperty->GetChildHandle(GET_MEMBER_NAME_CHECKED(FTickFunction, bStartWithTickEnabled)));
		TickCategory.AddProperty(PrimaryTickProperty->GetChildHandle(GET_MEMBER_NAME_CHECKED(FTickFunction, TickInterval)));
		TickCategory.AddProperty(PrimaryTickProperty->GetChildHandle(GET_MEMBER_NAME_CHECKED(FTickFunction, bTickEvenWhenPaused)), EPropertyLocation::Advanced);
		TickCategory.AddProperty(PrimaryTickProperty->GetChildHandle(GET_MEMBER_NAME_CHECKED(FTickFunction, bAllowTickOnDedicatedServer)), EPropertyLocation::Advanced);
		TickCategory.AddProperty(PrimaryTickProperty->GetChildHandle(GET_MEMBER_NAME_CHECKED(FTickFunction, TickGroup)), EPropertyLocation::Advanced);
	}

	PrimaryTickProperty->MarkHiddenByCustomization();
	DetailBuilder.HideCategory("BrushSettings");
	DetailBuilder.HideCategory("Navigation");
	DetailBuilder.HideCategory("Tags");
	DetailBuilder.HideCategory("Collision");
	DetailBuilder.HideCategory("HLOD");
	DetailBuilder.HideCategory("Mobile");
	DetailBuilder.HideCategory("Actor");

	const TArray<TWeakObjectPtr<UObject>> &SelectedObjects = DetailBuilder.GetSelectedObjects();
	for (int32 ObjectIndex = 0; ObjectIndex < SelectedObjects.Num(); ++ObjectIndex)
	{
		const TWeakObjectPtr<UObject>& CurrentObject = SelectedObjects[ObjectIndex];
		if (CurrentObject.IsValid())
		{
			ASVONavVolumeBase* CurrentVolume = Cast<ASVONavVolumeBase>(CurrentObject.Get());
			if (CurrentVolume != nullptr)
			{
				Volume = CurrentVolume;
				break;
			}
		}
	}

	DetailBuilder.EditCategory("SVONav")
		.AddCustomRow(NSLOCTEXT("NavVolume", "Build Octree", "Build Octree"))
		.ValueContent()
		.MaxDesiredWidth(125.f)
		.MinDesiredWidth(125.f)
		[
			SNew(SButton)
			.ContentPadding(2)
			.VAlign(VAlign_Center)
			.HAlign(HAlign_Center)
			.OnClicked(this, &FSVONavVolumeBaseProperties::OnBuildOctree)
		[
			SNew(STextBlock)
			.Font(IDetailLayoutBuilder::GetDetailFont())
			.Text(NSLOCTEXT("NavVolume", "Build Octree", "Build Octree"))
		]
		];

	DetailBuilder.EditCategory("SVONav")
		.AddCustomRow(NSLOCTEXT("NavVolume", "Clear Octree", "Clear Octree"))
		.ValueContent()
		.MaxDesiredWidth(125.f)
		.MinDesiredWidth(125.f)
		[
			SNew(SButton)
			.ContentPadding(2)
			.VAlign(VAlign_Center)
			.HAlign(HAlign_Center)
			.OnClicked(this, &FSVONavVolumeBaseProperties::OnClearOctree)
		[
			SNew(STextBlock)
			.Font(IDetailLayoutBuilder::GetDetailFont())
			.Text(NSLOCTEXT("NavVolume", "Clear Octree", "Clear Octree"))
		]
		];
}

FReply FSVONavVolumeBaseProperties::OnBuildOctree() const
{
	if (Volume.IsValid())
	{		
		Volume->BuildOctree();
	}
	return FReply::Handled();
}

FReply FSVONavVolumeBaseProperties::OnClearOctree() const
{
	if (Volume.IsValid())
	{
		Volume->Initialise();
	}
	return FReply::Handled();
}

#undef LOCTEXT_NAMESPACE
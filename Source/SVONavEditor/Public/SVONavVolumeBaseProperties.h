#pragma once

#include "CoreMinimal.h"
#include "IDetailCustomization.h"

class IDetailLayoutBuilder;
class ASVONavVolumeBase;

class FSVONavVolumeBaseProperties final : public IDetailCustomization
{
public:
	static TSharedRef<IDetailCustomization> MakeInstance();
	virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;
	FReply OnBuildOctree() const;
	FReply OnClearOctree() const;

private:
	TWeakObjectPtr<ASVONavVolumeBase> Volume;
};

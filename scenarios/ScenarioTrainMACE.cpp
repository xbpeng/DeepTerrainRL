#include "ScenarioTrainMACE.h"
#include "ScenarioExpMACE.h"
#include "util/FileUtil.h"

cScenarioTrainMACE::cScenarioTrainMACE()
{
	mTrainerParams.mPoolSize = 1;
	mInitExpBaseRate = 0.9;
}

cScenarioTrainMACE::~cScenarioTrainMACE()
{
}

std::string cScenarioTrainMACE::GetName() const
{
	return "Train MACE";
}

void cScenarioTrainMACE::BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer)
{
	int num_frags = 0;
	int frag_size = 0;
	GetFragParams(num_frags, frag_size);

	if (mEnableAsyncMode)
	{
		auto trainer = std::shared_ptr<cAsyncMACETrainer>(new cAsyncMACETrainer());
		trainer->SetNumActionFrags(num_frags);
		trainer->SetActionFragSize(frag_size);

		out_trainer = trainer;
	}
	else
	{
		auto trainer = std::shared_ptr<cMACETrainer>(new cMACETrainer());
		trainer->SetNumActionFrags(num_frags);
		trainer->SetActionFragSize(frag_size);

		out_trainer = trainer;
	}
}

void cScenarioTrainMACE::BuildExpScene(std::shared_ptr<cScenarioExp>& out_exp) const
{
	out_exp = std::shared_ptr<cScenarioExp>(new cScenarioExpMACE());
}

void cScenarioTrainMACE::GetFragParams(int& out_num_frags, int& out_frag_size) const
{
	auto ctrl = GetRefController();
	std::shared_ptr<cBaseControllerMACE> mace_ctrl = std::dynamic_pointer_cast<cBaseControllerMACE>(ctrl);

	if (mace_ctrl != nullptr)
	{
		out_num_frags = mace_ctrl->GetNumActionFrags();
		out_frag_size = mace_ctrl->GetActionFragSize();
	}
	else
	{
		assert(false); // controller does not implement MACE
	}
}

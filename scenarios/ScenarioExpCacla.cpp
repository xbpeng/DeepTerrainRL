#include "ScenarioExpCacla.h"
#include "learning/CaclaTrainer.h"

cScenarioExpCacla::cScenarioExpCacla()
{
}

cScenarioExpCacla::~cScenarioExpCacla()
{
}

std::string cScenarioExpCacla::GetName() const
{
	return "Exploration Cacla";
}

void cScenarioExpCacla::RecordFlagsBeg(tExpTuple& out_tuple) const
{
	const auto nn_ctrl = GetNNController();
	bool off_policy = nn_ctrl->IsOffPolicy();
	out_tuple.SetFlag(off_policy, cCaclaTrainer::eFlagOffPolicy);
}

void cScenarioExpCacla::RecordFlagsEnd(tExpTuple& out_tuple) const
{
	bool fail = CheckFail();
	out_tuple.SetFlag(fail, cCaclaTrainer::eFlagFail);
}

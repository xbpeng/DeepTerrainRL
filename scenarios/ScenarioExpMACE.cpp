#include "ScenarioExpMACE.h"

cScenarioExpMACE::cScenarioExpMACE()
{
}

cScenarioExpMACE::~cScenarioExpMACE()
{
}

std::string cScenarioExpMACE::GetName() const
{
	return "Exploration MACE";
}

void cScenarioExpMACE::RecordFlagsBeg(tExpTuple& out_tuple) const
{
	bool exp_critic = CheckExpCritic();
	bool exp_actor = CheckExpActor();
	out_tuple.SetFlag(exp_critic, cMACETrainer::eFlagExpCritic);
	out_tuple.SetFlag(exp_actor, cMACETrainer::eFlagExpActor);
}

void cScenarioExpMACE::RecordFlagsEnd(tExpTuple& out_tuple) const
{
	bool fail = CheckFail();
	out_tuple.SetFlag(fail, cMACETrainer::eFlagFail);
}

bool cScenarioExpMACE::CheckExpCritic() const
{
	bool exp = false;
	auto ctrl = GetCharacter()->GetController();
	std::shared_ptr<cBaseControllerMACE const> ac_int = std::dynamic_pointer_cast<cBaseControllerMACE const>(ctrl);

	if (ac_int != nullptr)
	{
		exp = ac_int->IsExpCritic();
	}
	else
	{
		assert(false); // controller does not implement actor-critic interface
	}

	return exp;
}

bool cScenarioExpMACE::CheckExpActor() const
{
	bool exp = false;
	auto ctrl = GetCharacter()->GetController();
	std::shared_ptr<cBaseControllerMACE const> ac_int = std::dynamic_pointer_cast<cBaseControllerMACE const>(ctrl);

	if (ac_int != nullptr)
	{
		exp = ac_int->IsExpActor();
	}
	else
	{
		assert(false); // controller does not implement actor-critic interface
	}

	return exp;
}
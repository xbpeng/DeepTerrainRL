#include "ScenarioExp.h"

#include <memory>
#include <ctime>
#include "sim/SimDog.h"
#include "sim/DogController.h"
#include "sim/DogControllerCacla.h"
#include "sim/DogControllerMACE.h"
#include "sim/GroundFlat.h"
#include "sim/GroundVar2D.h"

const int gNumWarmupCycles = 1;

cScenarioExp::cScenarioExp()
{
	mTupleBufferSize = 16;
	ResetTupleBuffer();
	ResetParams();

	mEnableExplore = true;
	mExpRate = 0.1;
	mExpBaseActionRate = 0.01;
}

cScenarioExp::~cScenarioExp()
{

}

void cScenarioExp::ParseArgs(const cArgParser& parser)
{
	cScenarioSimChar::ParseArgs(parser);
	parser.ParseString("policy_net", mPoliNetFile);
	parser.ParseString("policy_model", mPoliModelFile);
	parser.ParseString("critic_net", mCriticNetFile);
	parser.ParseString("critic_model", mCriticModelFile);

	parser.ParseInt("tuple_buffer_size", mTupleBufferSize);
	parser.ParseDouble("exp_rate", mExpRate);
	parser.ParseDouble("exp_temp", mExpTemp);
	parser.ParseDouble("exp_base_rate", mExpBaseActionRate);
}

void cScenarioExp::Init()
{
	cScenarioSimChar::Init();
	ResetParams();
	mTupleBuffer.resize(mTupleBufferSize);
	ResetTupleBuffer();
	
	EnableExplore(true);
	SetExpRate(mExpRate);
	SetExpTemp(mExpTemp);
	SetExpBaseActionRate(mExpBaseActionRate);

	if (EnableRandInitAction())
	{
		// start off with random action to get more diverse initial states
		CommandRandAction();
	}
}

void cScenarioExp::Reset()
{
	cScenarioSimChar::Reset();
	ResetParams();

	if (EnableRandInitAction())
	{
		// start off with random action to get more diverse initial states
		CommandRandAction();
	}
}

void cScenarioExp::Clear()
{
	cScenarioSimChar::Clear();
	ResetParams();
	mTupleBuffer.clear();
	ResetTupleBuffer();
}

void cScenarioExp::Update(double time_elapsed)
{
	cScenarioSimChar::Update(time_elapsed);

	if (time_elapsed > 0)
	{
		if (!IsNewCycle())
		{
			if (HasFallen())
			{
				NewCycleUpdate();
				Reset();
			}
		}
	}
}

void cScenarioExp::SetBufferSize(int size)
{
	mTupleBufferSize = size;
}

bool cScenarioExp::IsTupleBufferFull() const
{
	return mTupleCount >= mTupleBufferSize;
}

const std::vector<tExpTuple>& cScenarioExp::GetTuples() const
{
	return mTupleBuffer;
}

std::string cScenarioExp::GetName() const
{
	return "Exploration";
}

bool cScenarioExp::BuildController(std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = cScenarioSimChar::BuildController(out_ctrl);

	if (mPoliNetFile != "")
	{
		std::shared_ptr<cNNController> nn_ctrl = std::static_pointer_cast<cNNController>(out_ctrl);
		succ &= nn_ctrl->LoadNet(mPoliNetFile);

		if (succ && mPoliModelFile != "")
		{
			nn_ctrl->LoadModel(mPoliModelFile);
		}
	}

	return succ;
}


bool cScenarioExp::BuildDogControllerCacla(std::shared_ptr<cCharController>& out_ctrl) const
{
	bool succ = cScenarioSimChar::BuildDogControllerCacla(out_ctrl);
	std::shared_ptr<cDogControllerCacla> dog_ctrl = std::dynamic_pointer_cast<cDogControllerCacla>(out_ctrl);

	if (mCriticNetFile != "")
	{
		bool critic_succ = dog_ctrl->LoadCriticNet(mCriticNetFile);
		if (critic_succ && mCriticModelFile != "")
		{
			dog_ctrl->LoadCriticModel(mCriticModelFile);
		}
	}

	return succ;
}

void cScenarioExp::ResetParams()
{
	mCycleCount = 0;
}

void cScenarioExp::ResetTupleBuffer()
{
	mTupleCount = 0;
}

void cScenarioExp::EnableExplore(bool enable)
{
	mEnableExplore = enable;
	auto ctrl = mChar->GetController();
	ctrl->EnableExp(mEnableExplore);
}

void cScenarioExp::SetExpRate(double rate)
{
	mExpRate = rate;
	auto ctrl = mChar->GetController();
	ctrl->SetExpRate(rate);
}

void cScenarioExp::SetExpTemp(double temp)
{
	mExpTemp = temp;
	auto ctrl = mChar->GetController();
	ctrl->SetExpTemp(temp);
}

void cScenarioExp::SetExpBaseActionRate(double rate)
{
	mExpBaseActionRate = rate;
	auto ctrl = mChar->GetController();
	ctrl->SetExpBaseActionRate(rate);
}

double cScenarioExp::GetExpRate() const
{
	return mExpRate;
}

double cScenarioExp::GetExpTemp() const
{
	return mExpTemp;
}

double cScenarioExp::GetExpBaseActionRate() const
{
	return mExpBaseActionRate;
}

void cScenarioExp::PostSubstepUpdate(double time_step)
{
	if (IsNewCycle())
	{
		NewCycleUpdate();
	}
}

bool cScenarioExp::IsNewCycle() const
{
	const auto& ctrl = mChar->GetController();
	return ctrl->IsNewCycle();
}

void cScenarioExp::NewCycleUpdate()
{
	// finish recording tuple from previous cycle
	RecordState(mCurrTuple.mStateEnd);
	RecordFlagsEnd(mCurrTuple);
	mCurrTuple.mReward = CalcReward();

	// do something with the tuple
	if (IsValidTuple())
	{
		RecordTuple(mCurrTuple);
	}

	// start recording new tuple
	mCurrTuple.mStateBeg = mCurrTuple.mStateEnd;
	RecordAction(mCurrTuple.mAction);
	ClearFlags(mCurrTuple);
	RecordFlagsBeg(mCurrTuple);

	++mCycleCount;
}

std::shared_ptr<const cNNController> cScenarioExp::GetNNController() const
{
	return std::static_pointer_cast<const cNNController>(mChar->GetController());
}

std::shared_ptr<cNNController> cScenarioExp::GetNNController()
{
	return std::static_pointer_cast<cNNController>(mChar->GetController());
}

void cScenarioExp::RecordState(Eigen::VectorXd& out_state) const
{
	auto ctrl = GetNNController();
	ctrl->RecordPoliState(out_state);
}

void cScenarioExp::RecordAction(Eigen::VectorXd& out_action) const
{
	auto ctrl = GetNNController();
	ctrl->RecordPoliAction(out_action);
}

double cScenarioExp::CalcReward() const
{
	auto ctrl = GetNNController();
	double reward = ctrl->CalcReward();
	return reward;
}

bool cScenarioExp::CheckFail() const
{
	bool fail = HasFallen();
	return fail;
}


void cScenarioExp::ClearFlags(tExpTuple& out_tuple) const
{
	out_tuple.ClearFlags();
}

void cScenarioExp::RecordFlagsBeg(tExpTuple& out_tuple) const
{
}

void cScenarioExp::RecordFlagsEnd(tExpTuple& out_tuple) const
{
	bool fail = CheckFail();
	out_tuple.SetFlag(fail, cQNetTrainer::eFlagFail);
}

void cScenarioExp::RecordTuple(const tExpTuple& tuple)
{
	int idx = mTupleCount % mTupleBufferSize;
	mTupleBuffer[idx] = tuple;
	++mTupleCount;
}

bool cScenarioExp::EnableRandInitAction() const
{
	return true;
}

void cScenarioExp::CommandRandAction()
{
	const auto& ctrl = mChar->GetController();
	ctrl->CommandRandAction();
}

bool cScenarioExp::IsValidTuple() const
{
	bool valid = mCycleCount > gNumWarmupCycles;
	return valid;
}
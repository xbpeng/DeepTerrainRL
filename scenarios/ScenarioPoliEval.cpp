#include "ScenarioPoliEval.h"
#include "sim/NNController.h"
#include "sim/GroundVar2D.h"
#include "sim/DogControllerCacla.h"
#include "util/FileUtil.h"

const int gNumWarmupCycles = 1;

cScenarioPoliEval::cScenarioPoliEval()
{
	mPosStart.setZero();
	mAvgDist = 0;
	mEpisodeCount = 0;
	mCycleCount = 0;
	
	// analysis stuff
	mRecordNNActivation = false;
	mNNActivationOutputFile = "";
	mNNActivationLayer = "";

	mRecordActions = false;;
	mActionOutputFile = "";

	mRecordVel = false;
	mVelOutputFile = "";

	mRecordActionIDState = false;
	mActionIDStateOutputFile = "";

	mPrevCOMPos.setZero();
	mPrevTime = 0;
}

cScenarioPoliEval::~cScenarioPoliEval()
{
}

void cScenarioPoliEval::ParseArgs(const cArgParser& parser)
{
	cScenarioSimChar::ParseArgs(parser);
	parser.ParseString("policy_net", mPoliNetFile);
	parser.ParseString("policy_model", mPoliModelFile);
	parser.ParseString("critic_net", mCriticNetFile);
	parser.ParseString("critic_model", mCriticModelFile);

	parser.ParseBool("record_nn_activation", mRecordNNActivation);
	parser.ParseString("nn_activation_output_file", mNNActivationOutputFile);
	parser.ParseString("nn_activation_layer", mNNActivationLayer);

	parser.ParseBool("record_actions", mRecordActions);
	parser.ParseString("action_output_file", mActionOutputFile);

	parser.ParseBool("record_vel", mRecordVel);
	parser.ParseString("vel_output_file", mVelOutputFile);

	parser.ParseBool("record_action_id_state", mRecordActionIDState);
	parser.ParseString("action_id_state_output_file", mActionIDStateOutputFile);
}

void cScenarioPoliEval::Init()
{
	cScenarioSimChar::Init();
	mAvgDist = 0;
	mEpisodeCount = 0;
	mCycleCount = 0;
	mDistLog.clear();

	mPrevCOMPos = mChar->CalcCOM();
	mPrevTime = mTime;

	if (EnableRecordNNActivation())
	{
		InitNNActivation(mNNActivationOutputFile);
	}

	if (EnableRecordActions())
	{
		InitActionRecord(mActionOutputFile);
	}

	if (EnableRecordVel())
	{
		InitVelRecord(mVelOutputFile);
	}

	if (EnableRecordActionIDState())
	{
		InitActionIDState(mActionIDStateOutputFile);
	}
}

void cScenarioPoliEval::Reset()
{
	cScenarioSimChar::Reset();
	mPosStart = mChar->GetRootPos();

	mPrevCOMPos = mChar->CalcCOM();
	mPrevTime = mTime;
}

void cScenarioPoliEval::Clear()
{
	cScenarioSimChar::Clear();
	mAvgDist = 0;
	mEpisodeCount = 0;
	mCycleCount = 0;
	mDistLog.clear();
}

void cScenarioPoliEval::Update(double time_elapsed)
{
	cScenarioSimChar::Update(time_elapsed);

	if (time_elapsed > 0)
	{
		if (HasFallen())
		{
			if (IsValidCycle())
			{
				RecordDistTraveled();
			}
			Reset();
		}
	}
}

double cScenarioPoliEval::GetAvgDist() const
{
	return mAvgDist;
}

void cScenarioPoliEval::ResetAvgDist()
{
	mAvgDist = 0;
	mEpisodeCount = 0;
}

int cScenarioPoliEval::GetNumEpisodes() const
{
	return mEpisodeCount;
}

int cScenarioPoliEval::GetNumCycles() const
{
	return mCycleCount;
}

const std::vector<double>& cScenarioPoliEval::GetDistLog() const
{
	return mDistLog;
}

void cScenarioPoliEval::SetRandSeed(unsigned long seed)
{
	if ((typeid(*mGround.get()).hash_code() == typeid(cGroundVar2D).hash_code()))
	{
		std::shared_ptr<cGroundVar2D> ground = std::static_pointer_cast<cGroundVar2D>(mGround);
		ground->SeedRand(seed);
	}
}

std::string cScenarioPoliEval::GetName() const
{
	return "Policy Evaluation";
}

bool cScenarioPoliEval::BuildController(std::shared_ptr<cCharController>& out_ctrl)
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

bool cScenarioPoliEval::BuildDogControllerCacla(std::shared_ptr<cCharController>& out_ctrl) const
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

void cScenarioPoliEval::RecordDistTraveled()
{
	tVector curr_pos = mChar->GetRootPos();
	tVector delta = curr_pos - mPosStart;
	double dist = delta[0];

	mAvgDist = cMathUtil::AddAverage(mAvgDist, mEpisodeCount, dist, 1);
	++mEpisodeCount;

	mDistLog.push_back(dist);

#if defined (ENABLE_DEBUG_PRINT)
	printf("\nEpisodes: %i\n", mEpisodeCount);
	printf("Avg dist: %.5f\n", mAvgDist);
#endif
}

bool cScenarioPoliEval::IsNewCycle() const
{
	const auto& ctrl = mChar->GetController();
	return ctrl->IsNewCycle();
}

void cScenarioPoliEval::PostSubstepUpdate(double time_step)
{
	bool new_cycle = IsNewCycle();
	if (new_cycle)
	{
		NewCycleUpdate();
	}
}

void cScenarioPoliEval::NewCycleUpdate()
{
	if (IsValidCycle())
	{
		if (EnableRecordNNActivation())
		{
			RecordNNActivation(mNNActivationLayer, mNNActivationOutputFile);
		}

		if (EnableRecordActions())
		{
			RecordAction(mActionOutputFile);
		}

		if (EnableRecordVel())
		{
			RecordVel(mVelOutputFile);
		}

		if (EnableRecordActionIDState())
		{
			RecordActionIDState(mActionIDStateOutputFile);
		}
	}
	++mCycleCount;
}

void cScenarioPoliEval::InitNNActivation(const std::string& out_file)
{
	cFileUtil::ClearFile(out_file);
}

bool cScenarioPoliEval::EnableRecordNNActivation() const
{
	return mRecordNNActivation && mNNActivationLayer != "" && mNNActivationOutputFile != "";
}

void cScenarioPoliEval::RecordNNActivation(const std::string& layer_name, const std::string& out_file)
{
	const auto& ctrl = mChar->GetController();
	const auto& nn_ctrl = std::static_pointer_cast<cNNController const>(ctrl);
	const cNeuralNet& net = nn_ctrl->GetNet();

	Eigen::VectorXd data;
	net.GetLayerState(layer_name, data);

	int data_size = static_cast<int>(data.size());
	if (data_size > 0)
	{
		std::string data_str = "";
		int action_id = ctrl->GetCurrActionID();
		data_str += std::to_string(action_id);

		for (int i = 0; i < data_size; ++i)
		{
			data_str += ",\t";
			data_str += std::to_string(data[i]);
		}
		data_str += "\n";

		cFileUtil::AppendText(data_str, out_file);
	}
}

void cScenarioPoliEval::InitActionRecord(const std::string& out_file) const
{
	FILE* file = cFileUtil::OpenFile(out_file, "w");
	const auto& ctrl = mChar->GetController();

	int num_actions = ctrl->GetNumActions();
	Eigen::VectorXd params;
	for (int a = 0; a < num_actions; ++a)
	{
		ctrl->BuildActionOptParams(a, params);
		fprintf(file, "%i", a);

		int param_size = static_cast<int>(params.size());
		for (int i = 0; i < param_size; ++i)
		{
			fprintf(file, ", %.5f", params[i]);
		}
		fprintf(file, "\n");
	}

	cFileUtil::CloseFile(file);
}

bool cScenarioPoliEval::EnableRecordActions() const
{
	return mRecordActions && mActionOutputFile != "";
}

void cScenarioPoliEval::RecordAction(const std::string& out_file)
{
	const auto& ctrl = mChar->GetController();
	
	std::string data_str = "";
	int action_id = ctrl->GetCurrActionID();
	data_str += std::to_string(action_id);

	Eigen::VectorXd params;
	ctrl->BuildOptParams(params);

	int data_size = static_cast<int>(params.size());
	for (int i = 0; i < data_size; ++i)
	{
		data_str += ",\t";
		data_str += std::to_string(params[i]);
	}
	data_str += "\n";

	cFileUtil::AppendText(data_str, out_file);
}

void cScenarioPoliEval::InitVelRecord(const std::string& out_file) const
{
	cFileUtil::ClearFile(out_file);
}

bool cScenarioPoliEval::EnableRecordVel() const
{
	return mRecordVel && mVelOutputFile != "";
}

void cScenarioPoliEval::RecordVel(const std::string& out_file)
{
	tVector curr_com = mChar->CalcCOM();
	double curr_time = mTime;

	tVector vel = curr_com - mPrevCOMPos;
	double dt = curr_time - mPrevTime;
	vel /= dt;
	
	std::string str = std::to_string(vel[0]) + "\n";
	cFileUtil::AppendText(str, out_file);

	mPrevCOMPos = curr_com;
	mPrevTime = curr_time;
}

void cScenarioPoliEval::InitActionIDState(const std::string& out_file) const
{
	cFileUtil::ClearFile(out_file);
}

bool cScenarioPoliEval::EnableRecordActionIDState() const
{
	return mRecordActionIDState && mActionIDStateOutputFile != "";
}

void cScenarioPoliEval::RecordActionIDState(const std::string& out_file)
{
	const auto& ctrl = mChar->GetController();
	auto nn_ctrl = std::static_pointer_cast<const cNNController>(mChar->GetController());

	Eigen::VectorXd state;
	nn_ctrl->RecordPoliState(state);

	std::string data_str = "";
	int action_id = ctrl->GetCurrActionID();
	data_str += std::to_string(action_id);

	for (int i = 0; i < static_cast<int>(state.size()); ++i)
	{
		data_str += ",\t";
		data_str += std::to_string(state[i]);
	}
	data_str += "\n";

	cFileUtil::AppendText(data_str, out_file);
}

bool cScenarioPoliEval::IsValidCycle() const
{
	bool valid = mCycleCount >= gNumWarmupCycles;
	return valid;
}
#include "DogController.h"
#include <iostream>
#include <json/json.h>

#include "sim/SimCharacter.h"
#include "sim/RBDUtil.h"
#include "util/FileUtil.h"
#include "util/Util.h"

//#define DOG_CTRL_PROFILER

const cDogController::tStateDef gStateDefs[cDogController::eStateMax] =
{
	{
		"BackStance",
		true,								// mTransTime;
		cSimDog::eJointInvalid,				// mTransContact;
		cDogController::eStateExtend		// mNext;
	},
	{
		"Extend",
		false,								// mTransTime;
		cSimDog::eJointFinger,				// mTransFootContact;
		cDogController::eStateFrontStance	// mNext;
	},
	{
		"FrontStance",
		true,								// mTransTime;
		cSimDog::eJointInvalid,				// mTransContact;
		cDogController::eStateGather		// mNext;
	},
	{
		"Gather",
		false,								// mTransTime;
		cSimDog::eJointToe,					// mTransContact;
		cDogController::eStateInvalid		// mNext;
	},
};

const std::string gMiscParamsNames[cDogController::eMiscParamMax] = {
	"TransTime",
	"Cv",
	"BackForceX",
	"BackForceY",
	"FrontForceX",
	"FrontForceY"
};

const std::string gStateParamNames[cDogController::eStateParamMax] = {
	"SpineCurve",
	"Shoulder",
	"Elbow",
	"Hip",
	"Knee",
	"Ankle"
};

const cSimDog::eJoint gEndEffectors[] = {
	cSimDog::eJointToe,
	cSimDog::eJointFinger,
};
const int gNumEndEffectors = sizeof(gEndEffectors) / sizeof(gEndEffectors[0]);

const cSimDog::eJoint gSpineJoints[] = {
	cSimDog::eJointSpine0,
	cSimDog::eJointSpine1,
	cSimDog::eJointSpine2,
	cSimDog::eJointSpine3,
	cSimDog::eJointTorso
};
const int gNumSpineJoints = sizeof(gSpineJoints) / sizeof(gSpineJoints[0]);


const std::string gMiscParamsKey = "MiscParams";
const std::string gStateParamsKey = "StateParams";
const std::string gControllersKey = "Controllers";
const std::string gFilesKey = "Files";
const std::string gActionsKey = "Actions";


const cDogController::tParamInfo gParamInfo[] =
{
{false,	0, 1},			//TransTime
{true, -1, 1},			//Cv
{true, -1000, 1000},	//BackForceX
{true, -1000, 1000 },	//BackForceY
{true, -1000, 1000 },	//FrontForceX
{true, -1000, 1000 },	//FrontForceY

//BackStance
{true, -1, 1},			//SpineCurve
{true, -20, 20},		//Shoulder
{true, -20, 20 },		//Elbow
{true, -20, 20 },		//Hip
{true, -20, 20 },		//Knee
{true, -20, 20 },		//Ankle

//Extend
{ true, -1, 1 },		//SpineCurve
{ true, -20, 20 },		//Shoulder
{ true, -20, 20 },		//Elbow
{ true, -20, 20 },		//Hip
{ true, -20, 20 },		//Knee
{ true, -20, 20 },		//Ankle

//FrontStance
{ true, -1, 1 },		//SpineCurve
{ true, -20, 20 },		//Shoulder
{ true, -20, 20 },		//Elbow
{ true, -20, 20 },		//Hip
{ true, -20, 20 },		//Knee
{ true, -20, 20 },		//Ankle

//Gather
{ true, -1, 1 },		//SpineCurve
{ true, -20, 20 },		//Shoulder
{ true, -20, 20 },		//Elbow
{ true, -20, 20 },		//Hip
{ true, -20, 20 },		//Knee
{ true, -20, 20 }		//Ankle
};

/*
const cDogController::tParamInfo gParamInfo[] =
{
	{false,	0, 0.5},		//TransTime
	{true, -1, 1},			//Cv
	{true, -200, 1000},		//BackForceX
	{true, -200, 1000 },	//BackForceY
	{true, -200, 1000 },	//FrontForceX
	{true, -200, 1000 },	//FrontForceY

	//BackStance
	{true, -0.2, 1},		//SpineCurve
	{true, -10, 5},			//Shoulder
	{true, -1, 10 },		//Elbow
	{true, -5, 2 },			//Hip
	{true, -5, 5 },			//Knee
	{true, -2, 2 },			//Ankle

	//Extend
	{ true, -0.1, 0.5 },	//SpineCurve
	{ true, 0, 1 },			//Shoulder
	{ true, -1, 1 },		//Elbow
	{ true, 0, 2 },			//Hip
	{ true, -10, 0 },		//Knee
	{ true, 0, 10 },		//Ankle
	
	//FrontStance
	{ true, -0.2, 0.1 },	//SpineCurve
	{ true, -2, 0 },		//Shoulder
	{ true, 0, 2 },			//Elbow
	{ true, -2, 2 },		//Hip
	{ true, -3, 0 },		//Knee
	{ true, 0, 4 },			//Ankle
	
	//Gather
	{ true, -0.2, 0.1 },	//SpineCurve
	{ true, -3, 0 },		//Shoulder
	{ true, 0, 5 },			//Elbow
	{ true, -2, 2 },		//Hip
	{ true, -5, 0 },		//Knee
	{ true, 0, 5 }			//Ankle
};
*/
const int gNumParamInfo = sizeof(gParamInfo) / sizeof(gParamInfo[0]);

cDogController::cDogController() : cTerrainRLCharController()
{
	mDefaultAction = gInvalidIdx;
	mState = eStateBackStance;
	mEnableGravityCompensation = true;
}

cDogController::~cDogController()
{
}

void cDogController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
{
	// param_file should contain parameters for the pd controllers
	cTerrainRLCharController::Init(character);

	mGravity = gravity;
	mCtrlParams.clear();
	mPrevCOM = character->CalcCOM();

	Eigen::MatrixXd pd_params;
	bool succ = cPDController::LoadParams(param_file, pd_params);

	if (succ)
	{
		mRBDModel = std::shared_ptr<cRBDModel>(new cRBDModel());
		mRBDModel->Init(mChar->GetJointMat(), mChar->GetBodyDefs(), mGravity);

		mImpPDCtrl.Init(mChar, mRBDModel, pd_params, mGravity);
		
		succ = LoadControllers(param_file);
		TransitionState(eStateBackStance);
	}

	mValid = succ;
	if (!mValid)
	{
		printf("Failed to initialize dog controller\n");
		mValid = false;
	}
}

void cDogController::Reset()
{
	cTerrainRLCharController::Reset();
	ClearCommands();
	mImpPDCtrl.Reset();
	mPrevCOM = mChar->CalcCOM();
}

void cDogController::Clear()
{
	cTerrainRLCharController::Clear();
	mImpPDCtrl.Clear();
	mCtrlParams.clear();
	ClearCommands();

	mRBDModel.reset();
	mDefaultAction = gInvalidIdx;
}

void cDogController::Update(double time_step)
{
	cTerrainRLCharController::Update(time_step);

	int num_dof = mChar->GetNumDof();
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(num_dof);

	if (mMode == eModeActive)
	{
#if defined(DOG_CTRL_PROFILER)
		TIMER_PRINT_BEG(Dog_Ctrl_Update)
#endif
		mCurrCycleTime += time_step;
		UpdateStumbleCounter(time_step);

		UpdateRBDModel();
		UpdateState(time_step);
		ApplyFeedback();

		UpdatePDCtrls(time_step, tau);

		if (mEnableGravityCompensation)
		{
			ApplyGravityCompensation(tau);
		}

		ApplyVirtualForces(tau);

#if defined(DOG_CTRL_PROFILER)
		TIMER_PRINT_END(Dog_Ctrl_Update)
		printf("\n");
#endif
	}
	else if (mMode == eModePassive)
	{
		UpdatePDCtrls(time_step, tau);
	}

	mChar->ApplyControlForces(tau);
}

void cDogController::SeCtrlStateParams(eState state, const tStateParams& params)
{
	mCurrAction.mParams.segment(eMiscParamMax + state * eStateParamMax, eStateParamMax) = params;
}

void cDogController::TransitionState(int state)
{
	cTerrainRLCharController::TransitionState(state);
}

void cDogController::TransitionState(int state, double phase)
{
	assert(state >= 0 && state < eStateMax);
	cTerrainRLCharController::TransitionState(state, phase);
	tStateParams params = GetCurrParams();
	SetStateParams(params);
}

void cDogController::SetTransTime(double time)
{
	// set duration of each state in the walk
	mCurrAction.mParams[eMiscParamTransTime] = time;
}

int cDogController::GetNumStates() const
{
	return eStateMax;
}

void cDogController::SetMode(eMode mode)
{
	cTerrainRLCharController::SetMode(mode);

	if (mMode == eModePassive)
	{
		SetupPassiveMode();
	}
}

void cDogController::CommandAction(int action_id)
{
	int num_actions = GetNumActions();
	if (action_id < num_actions)
	{
		mCommands.push(action_id);
	}
	else
	{
		assert(false); // invalid action
	}
}

void cDogController::CommandRandAction()
{
	int num_actions = GetNumActions();
	int a = cMathUtil::RandInt(0, num_actions);
	CommandAction(a);
}

int cDogController::GetDefaultAction() const
{
	return mDefaultAction;
}

void cDogController::SetDefaultAction(int action_id)
{
	if (action_id >= 0 && action_id < GetNumActions())
	{
		mDefaultAction = action_id;
	}
}

int cDogController::GetNumActions() const
{
	return static_cast<int>(mActions.size());
}

void cDogController::BuildCtrlOptParams(int ctrl_params_idx, Eigen::VectorXd& out_params) const
{
	GetOptParams(mCtrlParams[ctrl_params_idx], out_params);
}

void cDogController::SetCtrlParams(int ctrl_params_id, const Eigen::VectorXd& params)
{
	assert(params.size() == GetNumParams());
	Eigen::VectorXd& ctrl_params = mCtrlParams[ctrl_params_id];
	ctrl_params = params;
	PostProcessParams(ctrl_params);

	const tBlendAction& curr_action = mActions[mCurrAction.mID];
	if (curr_action.mParamIdx0 == ctrl_params_id
		|| curr_action.mParamIdx1 == ctrl_params_id)
	{
		ApplyAction(mCurrAction.mID);
	}
}

void cDogController::SetCtrlOptParams(int ctrl_params_id, const Eigen::VectorXd& opt_params)
{
	assert(opt_params.size() == GetNumOptParams());
	Eigen::VectorXd& ctrl_params = mCtrlParams[ctrl_params_id];
	SetOptParams(opt_params, ctrl_params);

	const tBlendAction& curr_action = mActions[mCurrAction.mID];
	if (curr_action.mParamIdx0 == ctrl_params_id
		|| curr_action.mParamIdx1 == ctrl_params_id)
	{
		ApplyAction(mCurrAction.mID);
	}
}

void cDogController::BuildActionOptParams(int action_id, Eigen::VectorXd& out_params) const
{
	assert(action_id >= 0 && action_id < GetNumActions());
	const tBlendAction& action = mActions[action_id];
	Eigen::VectorXd params;
	BlendCtrlParams(action, params);
	GetOptParams(params, out_params);
}

int cDogController::GetNumParams() const
{
	int num_params = eMiscParamMax + eStateMax * eStateParamMax;
	return num_params;
}

int cDogController::GetNumOptParams() const
{
	int num_params = GetNumParams();
	assert(gNumParamInfo == num_params);

	int num_opt_params = 0;
	for (int i = 0; i < num_params; ++i)
	{
		if (IsOptParam(i))
		{
			++num_opt_params;
		}
	}

	return num_opt_params;
}

void cDogController::FetchOptParamScale(Eigen::VectorXd& out_scale) const
{
	int num_opt_params = GetNumOptParams();
	int num_params = GetNumParams();

	const double angle_scale = M_PI / 10;
	const double cv_scale = 0.02;
	const double cd_scale = 0.2;
	const double force_scale = 100;
	const double spine_curve_scale = 0.02;

	Eigen::VectorXd param_buffer = Eigen::VectorXd::Ones(num_params);

	int idx = 0;
	for (int i = 0; i < eMiscParamMax; ++i)
	{
		if (i == eMiscParamCv)
		{
			param_buffer[idx] = cv_scale;
		}
		else if (i == eMiscParamBackForceX
			|| i == eMiscParamBackForceY
			|| i == eMiscParamFrontForceX
			|| i == eMiscParamFrontForceY)
		{
			param_buffer[idx] = force_scale;
		}

		++idx;
	}

	for (int i = 0; i < eStateMax; ++i)
	{
		for (int j = 0; j < eStateParamMax; ++j)
		{
			if (j == eStateParamSpineCurve)
			{
				param_buffer[idx] = spine_curve_scale;
			}
			else
			{
				param_buffer[idx] = angle_scale;
			}
			++idx;
		}
	}

	assert(idx == num_params);
	GetOptParams(param_buffer, out_scale);
}

void cDogController::OutputOptParams(const std::string& file, const Eigen::VectorXd& params) const
{
	cController::OutputOptParams(file, params);
}

void cDogController::OutputOptParams(FILE* f, const Eigen::VectorXd& params) const
{
	std::string opt_param_json = BuildOptParamsJson(params);
	fprintf(f, "%s\n", opt_param_json.c_str());
}

void cDogController::ReadParams(std::ifstream& f_stream)
{
	Json::Reader reader;
	Json::Value root;
	bool succ = reader.parse(f_stream, root);
	if (succ)
	{
		double trans_time = 0;
		Eigen::VectorXd param_vec = Eigen::VectorXd::Zero(GetNumParams());
		int idx = 0;
		// grab the transition time first
		if (!root[gMiscParamsKey].isNull())
		{
			Json::Value misc_params = root[gMiscParamsKey];
			for (int i = 0; i < eMiscParamMax; ++i)
			{
				param_vec[idx++] = misc_params[gMiscParamsNames[i]].asDouble();
			}
		}
		else
		{
			printf("Failed to find %s\n", gMiscParamsKey.c_str());
		}

		if (!root[gStateParamsKey].isNull())
		{
			Json::Value state_params = root[gStateParamsKey];

			for (int i = 0; i < eStateMax; ++i)
			{
				Json::Value curr_params = state_params[gStateDefs[i].mName];
				for (int j = 0; j < eStateParamMax; ++j)
				{
					param_vec[idx++] = curr_params[gStateParamNames[j]].asDouble();
				}
			}
		}
		else
		{
			printf("Failed to find %s\n", gStateParamsKey.c_str());
		}

		if (!HasCtrlParams())
		{
			SetParams(param_vec);

			// refresh controls params
			tStateParams curr_params = GetCurrParams();
			SetStateParams(curr_params);
		}
		PostProcessParams(param_vec);
		mCtrlParams.push_back(param_vec);
	}
}

void cDogController::ReadParams(const std::string& file)
{
	cTerrainRLCharController::ReadParams(file);
}

void cDogController::SetParams(const Eigen::VectorXd& params)
{
	assert(params.size() == GetNumParams());
	mCurrAction.mParams = params;
	PostProcessParams(mCurrAction.mParams);
}

void cDogController::BuildOptParams(Eigen::VectorXd& out_params) const
{
	GetOptParams(mCurrAction.mParams, out_params);
}

void cDogController::SetOptParams(const Eigen::VectorXd& opt_params)
{
	SetOptParams(opt_params, mCurrAction.mParams);
}

void cDogController::SetOptParams(const Eigen::VectorXd& opt_params, Eigen::VectorXd& out_params) const
{
	assert(opt_params.size() == GetNumOptParams());
	assert(gNumParamInfo == GetNumParams());

	int num_params = GetNumParams();
	int opt_idx = 0;
	for (int i = 0; i < num_params; ++i)
	{
		if (IsOptParam(i))
		{
			out_params[i] = opt_params[opt_idx];
			++opt_idx;
		}
	}
	assert(opt_idx == GetNumOptParams());

	PostProcessParams(out_params);
}

void cDogController::BuildFromMotion(int ctrl_params_idx, const cMotion& motion)
{
	assert(motion.IsValid());
	double dur = motion.GetDuration();
	int num_states = GetNumStates();
	double trans_time = dur / num_states;
	SetTransTime(trans_time);

	Eigen::VectorXd params = mCurrAction.mParams;
	for (int s = 0; s < num_states; ++s)
	{
		double curr_time = (s + 1) * trans_time;
		Eigen::VectorXd curr_pose = motion.CalcFrame(curr_time);

		tStateParams state_params = tStateParams::Zero();
		BuildStateParamsFromPose(curr_pose, state_params);
		params.segment(eMiscParamMax + s * eStateParamMax, eStateParamMax) = state_params;
	}

	SetCtrlParams(ctrl_params_idx, params);
}

double cDogController::CalcReward() const
{
	tVector target_vel = GetTargetVel();
	
	const double vel_reward_w = 0.8;
	double vel_reward = 0;
	const double stumble_reward_w = 0.2;
	double stumble_reward = 0;

	bool fallen = mChar->HasFallen();
	if (!fallen)
	{
		double cycle_time = GetPrevCycleTime();
		double avg_vel = mPrevDistTraveled[0] / cycle_time;
		double vel_err = target_vel[0] - avg_vel;
		double vel_gamma = 0.5;
		vel_reward = std::exp(-vel_gamma * vel_err * vel_err);

		double stumble_count = mPrevStumbleCount;
		double avg_stumble = stumble_count /= cycle_time;
		double stumble_gamma = 10;
		stumble_reward = 1.0 / (1 + stumble_gamma * avg_stumble);
	}
	
	double reward = 0;
	reward += vel_reward_w * vel_reward
			+ stumble_reward_w * stumble_reward;

	return reward;
}

tVector cDogController::GetTargetVel() const
{
	return tVector(4, 0, 0, 0);
}

double cDogController::GetPrevCycleTime() const
{
	return mPrevCycleTime;
}

const tVector& cDogController::GetPrevDistTraveled() const
{
	return mPrevDistTraveled;
}

void cDogController::ResetParams()
{
	cTerrainRLCharController::ResetParams();
	mState = eStateBackStance;
	mPrevCycleTime = 0;
	mPrevDistTraveled.setZero();
	mCurrCycleTime = 0;
	mPrevCOM.setZero();
	mPrevStumbleCount = 0;
	mCurrStumbleCount = 0;
}

bool cDogController::LoadControllers(const std::string& file)
{
	std::ifstream f_stream(file);
	Json::Reader reader;
	Json::Value root;
	bool succ = reader.parse(f_stream, root);
	if (succ)
	{
		if (!root[gControllersKey].isNull())
		{
			Json::Value ctrl_root = root[gControllersKey];
			succ = ParseControllers(ctrl_root);
		}
	}

	if (succ)
	{
		if (mDefaultAction != gInvalidIdx)
		{
			ApplyAction(mDefaultAction);
		}
	}
	else
	{
		printf("failed to parse controllers from %s\n", file.c_str());
		assert(false);
	}

	return succ;
}

bool cDogController::ParseControllers(const Json::Value& root)
{
	bool succ = true;
	
	if (!root[gFilesKey].isNull())
	{
		succ &= ParseControllerFiles(root[gFilesKey]);
	}

	if (succ && !root[gActionsKey].isNull())
	{
		succ &= ParseActions(root[gActionsKey]);

		int num_action = GetNumActions();
		if (succ && num_action > 0)
		{
			mDefaultAction = 0;
			if (!root["DefaultAction"].isNull())
			{
				mDefaultAction = root["DefaultAction"].asInt();
				assert(mDefaultAction >= 0 && mDefaultAction < num_action);
			}
		}

		if (!root["EnableGravityCompensation"].isNull())
		{
			mEnableGravityCompensation = root["EnableGravityCompensation"].asBool();
		}
	}

	return succ;
}

bool cDogController::ParseControllerFiles(const Json::Value& root)
{
	bool succ = true;
	std::vector<std::string> files;
	assert(root.isArray());

	int num_files = root.size();
	files.resize(num_files);
	for (int f = 0; f < num_files; ++f)
	{
		std::string curr_file = root.get(f, 0).asString();
		ReadParams(curr_file);
	}
	return succ;
}

bool cDogController::ParseActions(const Json::Value& root)
{
	bool succ = true;
	assert(root.isArray());

	int num_actions = root.size();
	for (int a = 0; a < num_actions; ++a)
	{
		const Json::Value& curr_action = root.get(a, 0);
		tBlendAction action;
		succ &= ParseAction(curr_action, action);
		if (succ)
		{
			action.mID = static_cast<int>(mActions.size());
			mActions.push_back(action);
		}
		else
		{
			succ = false;
			break;
		}
	}

	if (!succ)
	{
		printf("failed to parse actions\n");
		assert(false);
	}
	return succ;
}

bool cDogController::ParseAction(const Json::Value& root, tBlendAction& out_action) const
{
	if (!root["ParamIdx0"].isNull())
	{
		out_action.mParamIdx0 = root["ParamIdx0"].asInt();
	}
	else
	{
		return false;
	}
	if (!root["ParamIdx1"].isNull())
	{
		out_action.mParamIdx1 = root["ParamIdx1"].asInt();
	}
	else
	{
		return false;
	}
	if (!root["Blend"].isNull())
	{
		out_action.mBlend = root["Blend"].asDouble();
	}
	else
	{
		return false;
	}
	if (!root["Cyclic"].isNull())
	{
		out_action.mCyclic = root["Cyclic"].asBool();
	}
	else
	{
		return false;
	}
	return true;
}

bool cDogController::HasCtrlParams() const
{
	return mCtrlParams.size() > 0;
}

void cDogController::UpdateState(double time_step)
{
	const cDogController::tStateDef& state = GetCurrStateDef();
	bool advance_state = mFirstCycle;

	double trans_time = GetTransTime();
	mPhase += time_step / trans_time;

	if (state.mTransTime)
	{
		if (mPhase >= 1)
		{
			advance_state = true;
		}
	}

	if (state.mTransContact != cSimDog::eJointInvalid)
	{
		bool contact = CheckContact(state.mTransContact);
		if (contact)
		{
			advance_state = true;
		}
	}

	if (advance_state)
	{
		eState next_state = (mFirstCycle) ? eStateBackStance : state.mNext;
		bool end_step = (next_state == eStateInvalid) || mFirstCycle;

		if (end_step)
		{
			UpdateAction();
			mFirstCycle = false;
		}
		else
		{
			TransitionState(next_state);
		}
	}
}

void cDogController::UpdateAction()
{
	ParseGround();
	BuildPoliState(mPoliState);

	mIsOffPolicy = true;

	if (HasCommands())
	{
		ProcessCommand(mCurrAction);
	}
	else if (HasNet())
	{
		DecideAction(mCurrAction);
	}
	else if (!IsCurrActionCyclic())
	{
		BuildDefaultAction(mCurrAction);
	}

	ApplyAction(mCurrAction);
}

void cDogController::UpdateRBDModel()
{
	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	mChar->BuildPose(pose);
	mChar->BuildVel(vel);

#if defined(DOG_CTRL_PROFILER)
	TIMER_PRINT_BEG(Update_RBD_Model)
#endif

	mRBDModel->Update(pose, vel);
	cRBDUtil::BuildJacobian(*mRBDModel.get(), mJacobian);
	
#if defined(DOG_CTRL_PROFILER)
	TIMER_PRINT_END(Update_RBD_Model)
#endif
}

void cDogController::UpdatePDCtrls(double time_step, Eigen::VectorXd& out_tau)
{
	mImpPDCtrl.UpdateControlForce(time_step, out_tau);
}

void cDogController::UpdateStumbleCounter(double time_step)
{
	bool stumbled = mChar->HasStumbled();
	if (stumbled)
	{
		mCurrStumbleCount += time_step;
	}
}

void cDogController::ApplyFeedback()
{
	const cSimDog::eJoint joints[] =
	{
		cSimDog::eJointHip,
		cSimDog::eJointShoulder
	};
	const int num_joints = sizeof(joints) / sizeof(joints[0]);

	const cSimDog::eJoint end_effectors[] =
	{
		cSimDog::eJointToe,
		cSimDog::eJointFinger
	};

	const eStateParam params[] = 
	{
		eStateParamHip,
		eStateParamShoulder
	};
	assert(sizeof(end_effectors) / sizeof(end_effectors[0]) == num_joints);
	assert(sizeof(params) / sizeof(params[0]) == num_joints);

	tVector com_vel = mChar->CalcCOMVel();
	for (int j = 0; j < num_joints; ++j)
	{
		cSimDog::eJoint joint_id = joints[j];
		cSimDog::eJoint eff_id = end_effectors[j];
		const auto& body_part = mChar->GetBodyPart(eff_id);
		
		bool contact = body_part->IsInContact();
		if (!contact)
		{
			double vel_gain = GetCv();
			eStateParam param_id = params[j];
			double delta_theta = com_vel[0] * vel_gain;
			
			double default_theta = GetCurrParams()[param_id];
			double corrected_theta = default_theta + delta_theta;
			mImpPDCtrl.SetTargetTheta(joint_id, corrected_theta);
		}
	}
}

void cDogController::ApplyGravityCompensation(Eigen::VectorXd& out_tau)
{
#if defined(DOG_CTRL_PROFILER)
	TIMER_PRINT_BEG(Gravity_Comp)
#endif

	const double lambda = 0.0001;
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	const Eigen::MatrixXd& body_defs = mChar->GetBodyDefs();
	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	mChar->BuildPose(pose);
	vel = Eigen::VectorXd::Zero(pose.size());

	bool has_support = false;
	Eigen::MatrixXd contact_basis = BuildContactBasis(pose, has_support);
	
	if (has_support)
	{
		Eigen::VectorXd tau_g;
		cRBDUtil::CalcGravityForce(*mRBDModel.get(), tau_g);
		tau_g = -tau_g;

		int basis_cols = static_cast<int>(contact_basis.cols());

		int root_id = mChar->GetRootID();
		int root_offset = mChar->GetParamOffset(root_id);
		int root_size = mChar->GetParamSize(root_id);

		Eigen::VectorXd b = tau_g.segment(root_offset, root_size);
		Eigen::MatrixXd A = contact_basis.block(root_offset, 0, root_size, basis_cols);

		Eigen::MatrixXd AtA = A.transpose() * A;
		Eigen::VectorXd Atb = A.transpose() * b;
		AtA.diagonal() += lambda * Eigen::VectorXd::Ones(basis_cols);

		Eigen::VectorXd x = (AtA).householderQr().solve(Atb);
		Eigen::VectorXd tau_contact = contact_basis * x;

		tau_g -= tau_contact;
		tau_g.segment(root_offset, root_size).setZero();

		out_tau += tau_g;

#if defined(DOG_CTRL_PROFILER)
		TIMER_PRINT_END(Gravity_Comp)
#endif
	}
}

void cDogController::ApplyVirtualForces(Eigen::VectorXd& out_tau)
{
	for (int e = 0; e < gNumEndEffectors; ++e)
	{
		cSimDog::eJoint joint_id = gEndEffectors[e];
		bool active_effector = IsActiveVFEffector(joint_id);
		
		if (active_effector)
		{
			tVector vf = -GetEffectorVF(joint_id);
			
			tVector pos = GetEndEffectorContactPos(joint_id);
			cSpAlg::tSpTrans joint_world_trans = cSpAlg::BuildTrans(-pos);

			cSpAlg::tSpVec sp_force = cSpAlg::BuildSV(vf);
			sp_force = cSpAlg::ApplyTransF(joint_world_trans, sp_force);

			const Eigen::MatrixXd& joint_mat = mRBDModel->GetJointMat();
			int curr_id = joint_id;
			while (curr_id != cSimDog::eJointRoot
				&& curr_id != cSimDog::eJointTorso)
			{
				assert(curr_id != cKinTree::gInvalidJointID);
				int offset = cKinTree::GetParamOffset(joint_mat, curr_id);
				int size = cKinTree::GetParamSize(joint_mat, curr_id);
				const auto curr_J = mJacobian.block(0, offset, cSpAlg::gSpVecSize, size);

				out_tau.segment(offset, size) += curr_J.transpose() * sp_force;
				curr_id = mRBDModel->GetParent(curr_id);
			}
		}
	}
}

const cDogController::tStateDef& cDogController::GetCurrStateDef() const
{
	return gStateDefs[mState];
}

cDogController::tStateParams cDogController::GetCurrParams() const
{
	tStateParams params = mCurrAction.mParams.segment(eMiscParamMax + mState * eStateParamMax, eStateParamMax);
	return params;
}

void cDogController::SetStateParams(const tStateParams& params)
{
	for (int i = 0; i < gNumSpineJoints; ++i)
	{
		cSimDog::eJoint joint = gSpineJoints[i];
		mImpPDCtrl.SetTargetTheta(joint, params(eStateParamSpineCurve));
	}
	mImpPDCtrl.SetTargetTheta(cSimDog::eJointShoulder, params(eStateParamShoulder));
	mImpPDCtrl.SetTargetTheta(cSimDog::eJointElbow, params(eStateParamElbow));
	mImpPDCtrl.SetTargetTheta(cSimDog::eJointHip, params(eStateParamHip));
	mImpPDCtrl.SetTargetTheta(cSimDog::eJointKnee, params(eStateParamKnee));
	mImpPDCtrl.SetTargetTheta(cSimDog::eJointAnkle, params(eStateParamAnkle));
}

void cDogController::SetupPassiveMode()
{
	tStateParams params;
	params.setZero();
	SetStateParams(params);
}

double cDogController::GetTransTime() const
{
	return mCurrAction.mParams[eMiscParamTransTime];
}

double cDogController::GetCv() const
{
	return mCurrAction.mParams[eMiscParamCv];
}

tVector cDogController::GetBackForce() const
{
	return tVector(mCurrAction.mParams[eMiscParamBackForceX], mCurrAction.mParams[eMiscParamBackForceY], 0, 0);
}

tVector cDogController::GetFrontForce() const
{
	return tVector(mCurrAction.mParams[eMiscParamFrontForceX], mCurrAction.mParams[eMiscParamFrontForceY], 0, 0);
}

bool cDogController::CheckContact(cSimDog::eJoint joint_id) const
{
	assert(joint_id != cSimDog::eJointInvalid);
	const auto& body_part = mChar->GetBodyPart(joint_id);
	bool contact = body_part->IsInContact();
	return contact;
}

bool cDogController::IsActiveVFEffector(cSimDog::eJoint joint_id) const
{
	bool valid_effector = ((mState == eStateBackStance || mState == eStateExtend)
		&& joint_id == cSimDog::eJointToe)
		|| ((mState == eStateFrontStance || mState == eStateGather)
			&& joint_id == cSimDog::eJointFinger);

	bool active_effector = valid_effector && CheckContact(joint_id);
	return active_effector;
}

tVector cDogController::GetEffectorVF(cSimDog::eJoint joint_id) const
{
	tVector force = tVector::Zero();
	switch (joint_id)
	{
	case cSimDog::eJointToe:
		force = GetBackForce();
		break;
	case cSimDog::eJointFinger:
		force = GetFrontForce();
		break;
	default:
		assert(false); // unsupported effector
		break;
	}
	return force;
}

Eigen::MatrixXd cDogController::BuildContactBasis(const Eigen::VectorXd& pose, bool& out_has_support) const
{
	const int num_basis = 2;
	int rows = static_cast<int>(pose.size());
	const int cols = gNumEndEffectors * num_basis;

	const Eigen::Matrix<double, 6, num_basis> force_svs
		((Eigen::Matrix<double, 6, num_basis>() << 
			0, 0,
			0, 0,
			0, 0,
			0, 1,
			1, 0,
			0, 0).finished());

	Eigen::MatrixXd contact_basis = Eigen::MatrixXd::Zero(rows, cols);

	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	tVector root_pos = mChar->GetRootPos();

	out_has_support = false;
	for (int e = 0; e < gNumEndEffectors; ++e)
	{
		cSimDog::eJoint joint_id = gEndEffectors[e];
		bool valid_support = CheckContact(joint_id);

		auto curr_block = contact_basis.block(0, e * num_basis, rows, num_basis);
		if (valid_support)
		{
			out_has_support = true;

			tVector pos = GetEndEffectorContactPos(joint_id);
			cSpAlg::tSpTrans joint_world_trans = cSpAlg::BuildTrans(-pos);

			Eigen::Matrix<double, 6, num_basis> force_basis;
			for (int i = 0; i < num_basis; ++i)
			{
				cSpAlg::tSpVec force_sv = force_svs.col(i);
				force_basis.col(i) = cSpAlg::ApplyTransF(joint_world_trans, force_sv);
			}

			int curr_id = joint_id;
			while (curr_id != cKinTree::gInvalidJointID)
			{
				int offset = cKinTree::GetParamOffset(joint_mat, curr_id);
				int size = cKinTree::GetParamSize(joint_mat, curr_id);
				const auto curr_J = mJacobian.block(0, offset, cSpAlg::gSpVecSize, size);

				curr_block.block(offset, 0, size, curr_block.cols()) = curr_J.transpose() * force_basis;
				curr_id = mRBDModel->GetParent(curr_id);
			}
		}
	}

	return contact_basis;
}

const std::string& cDogController::GetStateName(eState state) const
{
	return gStateDefs[state].mName;
}

const std::string& cDogController::GetStateParamName(eStateParam param) const
{
	return gStateParamNames[param];
}

void cDogController::GetOptParams(const Eigen::VectorXd& ctrl_params, Eigen::VectorXd& out_opt_params) const
{
	int num_params = GetNumParams();
	int num_opt_params = GetNumOptParams();
	assert(ctrl_params.size() == num_params);
	assert(gNumParamInfo == num_params);

	out_opt_params.resize(num_opt_params);

	int opt_idx = 0;
	for (int i = 0; i < num_params; ++i)
	{
		if (IsOptParam(i))
		{
			out_opt_params[opt_idx] = ctrl_params[i];
			++opt_idx;
		}
	}
	assert(opt_idx == num_opt_params);
}

std::string cDogController::BuildOptParamsJson(const Eigen::VectorXd& opt_params) const
{
	assert(opt_params.size() == GetNumOptParams());
	Eigen::VectorXd param_buffer = mCurrAction.mParams;
	SetOptParams(opt_params, param_buffer);
	std::string json = BuildParamsJson(param_buffer);
	return json;
}

std::string cDogController::BuildParamsJson(const Eigen::VectorXd& params) const
{
	std::string json = "";
	int idx = 0;

	json += "\"" + gMiscParamsKey + "\": \n{\n";
	for (int i = 0; i < eMiscParamMax; ++i)
	{
		if (i != 0)
		{
			json += ",\n";
		}
		json += "\"" + gMiscParamsNames[i] + "\": " + std::to_string(params[idx++]);
	}
	json += "\n},\n\n";

	json += "\"" + gStateParamsKey + "\": \n{\n";
	for (int i = 0; i < eStateMax; ++i)
	{
		if (i != 0)
		{
			json += ",\n";
		}

		json += "\"" + gStateDefs[i].mName + "\": \n{\n";
		for (int j = 0; j < eStateParamMax; ++j)
		{
			if (j != 0)
			{
				json += ",\n";
			}
			json += "\"" + gStateParamNames[j] + "\": " + std::to_string(params[idx++]);
		}
		json += "\n}";
	}
	json += "\n}\n";

	json = "{" + json + "}";
	return json;
}

void cDogController::BuildStateParamsFromPose(const Eigen::VectorXd& pose, tStateParams& out_params)
{
	const cSimDog::eJoint ctrl_joints[] =
	{
		cSimDog::eJointShoulder,
		cSimDog::eJointElbow,
		cSimDog::eJointHip,
		cSimDog::eJointKnee,
		cSimDog::eJointAnkle
	};
	const int num_ctrl_joints = sizeof(ctrl_joints) / sizeof(ctrl_joints[0]);

	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();

	out_params(eStateParamSpineCurve) = CalcSpineCurve(pose);

	for (int i = 0; i < num_ctrl_joints; ++i)
	{
		tVector axis;
		double tar_theta = 0;
		int joint_id = ctrl_joints[i];
		if (mImpPDCtrl.UseWorldCoord(joint_id))
		{
			cKinTree::CalcJointWorldTheta(joint_mat, pose, joint_id, axis, tar_theta);
		}
		else
		{
			tar_theta = cKinTree::GetJointTheta(joint_mat, pose, joint_id);
		}
		out_params(eStateParamShoulder + i) = tar_theta;
	}
}

double cDogController::CalcSpineCurve(const Eigen::VectorXd& pose) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();

	double curve = 0;
	for (int i = 0; i < gNumSpineJoints; ++i)
	{
		cSimDog::eJoint joint = gSpineJoints[i];
		double theta = cKinTree::GetJointTheta(joint_mat, pose, joint);
		curve += theta;
	}

	curve /= gNumSpineJoints;
	return curve;
}

const Eigen::VectorXd& cDogController::GetCtrlParams(int ctrl_id) const
{
	return mCtrlParams[ctrl_id];
}

bool cDogController::IsCurrActionCyclic() const
{
	const tBlendAction& action = mActions[mCurrAction.mID];
	return action.mCyclic;
}

void cDogController::ApplyAction(int action_id)
{
	cTerrainRLCharController::ApplyAction(action_id);
}

void cDogController::ApplyAction(const tAction& action)
{
	cTerrainRLCharController::ApplyAction(action);
	TransitionState(eStateBackStance);
}

void cDogController::NewCycleUpdate()
{
	cTerrainRLCharController::NewCycleUpdate();
	mPrevCycleTime = mCurrCycleTime;
	mCurrCycleTime = 0;
	mPrevStumbleCount = mCurrStumbleCount;
	mCurrStumbleCount = 0;
	RecordDistTraveled();
	mPrevCOM = mChar->CalcCOM();
}

void cDogController::BlendCtrlParams(const tBlendAction& action, Eigen::VectorXd& out_params) const
{
	const Eigen::VectorXd& param0 = GetCtrlParams(action.mParamIdx0);
	const Eigen::VectorXd& param1 = GetCtrlParams(action.mParamIdx1);
	double blend = action.mBlend;
	out_params = (1 - blend) * param0 + blend * param1;
}

void cDogController::PostProcessParams(Eigen::VectorXd& out_params) const
{
	out_params[eMiscParamTransTime] = std::abs(out_params[eMiscParamTransTime]);
	out_params[eMiscParamCv] = std::abs(out_params[eMiscParamCv]);
}

bool cDogController::IsOptParam(int param_idx) const
{
	assert(param_idx >= 0 && param_idx < gNumParamInfo);
	return gParamInfo[param_idx].mIsOptParam;
}

double cDogController::GetParamBoundMin(int param_idx) const
{
	assert(param_idx >= 0 && param_idx < gNumParamInfo);
	return gParamInfo[param_idx].mMin;
}

double cDogController::GetParamBoundMax(int param_idx) const
{
	assert(param_idx >= 0 && param_idx < gNumParamInfo);
	return gParamInfo[param_idx].mMax;
}

tVector cDogController::GetEndEffectorContactPos(int joint_id) const
{
	tVector pos = tVector::Zero();
	const auto& part = mChar->GetBodyPart(joint_id);
	if (part != nullptr)
	{
		const auto& body_defs = mChar->GetBodyDefs();
		tVector size = cKinTree::GetBodySize(body_defs, joint_id);
		pos = part->LocalToWorldPos(tVector(0, -0.5 * size[1], 0, 0));
	}
	else
	{
		pos = mRBDModel->CalcJointWorldPos(joint_id);
	}
	return pos;
}

void cDogController::RecordDistTraveled()
{
	tVector com = mChar->CalcCOM();
	mPrevDistTraveled = com - mPrevCOM;
}

int cDogController::PopCommand()
{
	int cmd = mCommands.top();
	mCommands.pop();
	return cmd;
}

bool cDogController::HasCommands() const
{
	return !mCommands.empty();
}

void cDogController::ClearCommands()
{
	while (!mCommands.empty())
	{
		mCommands.pop();
	}
}

void cDogController::ProcessCommand(tAction& out_action)
{
	int a_id = PopCommand();
	BuildBaseAction(a_id, out_action);
}

void cDogController::BuildBaseAction(int action_id, tAction& out_action) const
{
	const tBlendAction& blend = mActions[action_id];
	out_action.mID = blend.mID;
	BlendCtrlParams(blend, out_action.mParams);
}
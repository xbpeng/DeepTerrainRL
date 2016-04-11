#include "RaptorController.h"
#include <iostream>
#include <ctime>
#include <json/json.h>

#include "sim/SimCharacter.h"
#include "sim/RBDUtil.h"
#include "util/FileUtil.h"

const cRaptorController::eStance gDefaultStance = cRaptorController::eStanceRight;
const cRaptorController::tStateDef gStateDefs[cRaptorController::eStateMax] =
{
	{
		"Contact",
		true,									// mTransTime;
		false,									// mTransContact;
		cRaptorController::eStateDown			// mNext;
	},
	{
		"Down",
		true,									// mTransTime;
		false,									// mTransFootContact;
		cRaptorController::eStatePassing		// mNext;
	},
	{
		"Passing",
		true,									// mTransTime;
		false,									// mTransContact;
		cRaptorController::eStateUp				// mNext;
	},
	{
		"Up",
		false,									// mTransTime;
		true,									// mTransContact;
		cRaptorController::eStateInvalid		// mNext;
	},
};

const std::string gMiscParamsNames[cRaptorController::eMiscParamMax] = {
	"TransTime",
	"Cv",
	"Cd",
	"ForceX",
	"ForceY"
};

const std::string gStateParamNames[cRaptorController::eStateParamMax] = {
	"RootPitch",
	"SpineCurve",
	"StanceHip",
	"StanceKnee",
	"StanceAnkle",
	"SwingHip",
	"SwingKnee",
	"SwingAnkle"
};

const cSimRaptor::eJoint gEndEffectors[] = {
	cSimRaptor::eJointRightToe,
	cSimRaptor::eJointLeftToe,
};
const int gNumEndEffectors = sizeof(gEndEffectors) / sizeof(gEndEffectors[0]);

const cSimRaptor::eJoint gSpineJoints[] = {
	cSimRaptor::eJointSpine0,
	cSimRaptor::eJointSpine1,
	cSimRaptor::eJointSpine2,
	cSimRaptor::eJointSpine3
};
const int gNumSpineJoints = sizeof(gSpineJoints) / sizeof(gSpineJoints[0]);

const std::string gMiscParamsKey = "MiscParams";
const std::string gStateParamsKey = "StateParams";
const std::string gControllersKey = "Controllers";
const std::string gFilesKey = "Files";
const std::string gActionsKey = "Actions";

const bool gOptParamsMasks[] =
{
	false,	//TransTime
	true,	//Cv
	true,	//Cd
	false,	//ForceX
	false,	//ForceY

	//Contact
	true,	//RootPitch
	false,	//SpineCurve
	true,	//StanceHip
	true,	//StanceKnee
	true,	//StanceAnkle
	true,	//SwingHip
	true,	//SwingKnee
	true,	//SwingAnkle

	//Down
	true,	//RootPitch
	false,	//SpineCurve
	true,	//StanceHip
	true,	//StanceKnee
	true,	//StanceAnkle
	true,	//SwingHip
	true,	//SwingKnee
	true,	//SwingAnkle

	//Passing
	false,	//RootPitch
	false,	//SpineCurve
	true,	//StanceHip
	true,	//StanceKnee
	true,	//StanceAnkle
	true,	//SwingHip
	true,	//SwingKnee
	true,	//SwingAnkle

	//Up
	false,	//RootPitch
	false,	//SpineCurve
	true,	//StanceHip
	true,	//StanceKnee
	true,	//StanceAnkle
	true,	//SwingHip
	true,	//SwingKnee
	true	//SwingAnkle
};
const int gNumOptParamMasks = sizeof(gOptParamsMasks) / sizeof(gOptParamsMasks[0]);

cRaptorController::cRaptorController() : cTerrainRLCharController()
{
	mDefaultAction = gInvalidIdx;
	mState = eStateContact;
	mEnableGravityCompensation = true;
	mEnableVirtualForces = true;
}

cRaptorController::~cRaptorController()
{
}

void cRaptorController::Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file)
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
		TransitionState(eStateContact);
	}

	if (succ)
	{
		mValid = true;
		SetStance(gDefaultStance);
	}
	else
	{
		printf("Failed to initialize dog controller\n");
		mValid = false;
	}
}

void cRaptorController::Reset()
{
	cTerrainRLCharController::Reset();
	ClearCommands();
	mImpPDCtrl.Reset();
	SetStance(gDefaultStance);
	mPrevCOM = mChar->CalcCOM();
}

void cRaptorController::Clear()
{
	cTerrainRLCharController::Clear();
	mImpPDCtrl.Clear();
	mCtrlParams.clear();
	ClearCommands();

	mRBDModel.reset();
	mDefaultAction = gInvalidIdx;
}

void cRaptorController::Update(double time_step)
{
	cTerrainRLCharController::Update(time_step);

	int num_dof = mChar->GetNumDof();
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(num_dof);

	if (mMode == eModeActive)
	{
		mCurrCycleTime += time_step;
		UpdateStumbleCounter(time_step);
		
		UpdateRBDModel();
		UpdateState(time_step);
		UpdateStanceHip();

		// order matters!
		ApplySwingFeedback(tau);
		UpdatePDCtrls(time_step, tau);

		if (mEnableGravityCompensation)
		{
			ApplyGravityCompensation(tau);
		}
		
		ApplyStanceFeedback(tau);

		if (mEnableVirtualForces)
		{
			ApplyVirtualForces(tau);
		}
	}
	else if (mMode == eModePassive)
	{
		UpdatePDCtrls(time_step, tau);
	}

	mChar->ApplyControlForces(tau);
}

void cRaptorController::SeCtrlStateParams(eState state, const tStateParams& params)
{
	mCurrAction.mParams.segment(eMiscParamMax + state * eStateParamMax, eStateParamMax) = params;
}

void cRaptorController::TransitionState(int state)
{
	cTerrainRLCharController::TransitionState(state);
}

void cRaptorController::TransitionState(int state, double phase)
{
	assert(state >= 0 && state < eStateMax);
	cTerrainRLCharController::TransitionState(state, phase);

	tStateParams params = GetCurrParams();
	SetStateParams(params);
}

void cRaptorController::SetTransTime(double time)
{
	// set duration of each state in the walk
	mCurrAction.mParams[eMiscParamTransTime] = time;
}

int cRaptorController::GetNumStates() const
{
	return eStateMax;
}

void cRaptorController::SetMode(eMode mode)
{
	cTerrainRLCharController::SetMode(mode);

	if (mMode == eModePassive)
	{
		SetupPassiveMode();
	}
}

void cRaptorController::CommandAction(int action_id)
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

void cRaptorController::CommandRandAction()
{
	int num_actions = GetNumActions();
	int a = cMathUtil::RandInt(0, num_actions);
	CommandAction(a);
}

int cRaptorController::GetDefaultAction() const
{
	return mDefaultAction;
}

void cRaptorController::SetDefaultAction(int action_id)
{
	if (action_id >= 0 && action_id < GetNumActions())
	{
		mDefaultAction = action_id;
	}
}

int cRaptorController::GetNumActions() const
{
	return static_cast<int>(mActions.size());
}

void cRaptorController::BuildCtrlOptParams(int ctrl_params_idx, Eigen::VectorXd& out_params) const
{
	GetOptParams(mCtrlParams[ctrl_params_idx], out_params);
}

void cRaptorController::SetCtrlParams(int ctrl_params_id, const Eigen::VectorXd& params)
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

void cRaptorController::SetCtrlOptParams(int ctrl_params_id, const Eigen::VectorXd& opt_params)
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

void cRaptorController::BuildActionOptParams(int action_id, Eigen::VectorXd& out_params) const
{
	assert(action_id >= 0 && action_id < GetNumActions());
	const tBlendAction& action = mActions[action_id];
	Eigen::VectorXd params;
	BlendCtrlParams(action, params);
	GetOptParams(params, out_params);
}

int cRaptorController::GetNumParams() const
{
	int num_params = eMiscParamMax + eStateMax * eStateParamMax;
	return num_params;
}

int cRaptorController::GetNumOptParams() const
{
	int num_params = GetNumParams();
	assert(gNumOptParamMasks == num_params);
	
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

void cRaptorController::FetchOptParamScale(Eigen::VectorXd& out_scale) const
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
		else if (i == eMiscParamCd)
		{
			param_buffer[idx] = cd_scale;
		}
		else if (i == eMiscParamForceX || i == eMiscParamForceY)
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

void cRaptorController::OutputOptParams(const std::string& file, const Eigen::VectorXd& params) const
{
	cController::OutputOptParams(file, params);
}

void cRaptorController::OutputOptParams(FILE* f, const Eigen::VectorXd& params) const
{
	std::string opt_param_json = BuildOptParamsJson(params);
	fprintf(f, "%s\n", opt_param_json.c_str());
}

void cRaptorController::ReadParams(std::ifstream& f_stream)
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

void cRaptorController::ReadParams(const std::string& file)
{
	cTerrainRLCharController::ReadParams(file);
}

void cRaptorController::SetParams(const Eigen::VectorXd& params)
{
	assert(params.size() == GetNumParams());
	mCurrAction.mParams = params;
	PostProcessParams(mCurrAction.mParams);
}

void cRaptorController::BuildOptParams(Eigen::VectorXd& out_params) const
{
	GetOptParams(mCurrAction.mParams, out_params);
}

void cRaptorController::SetOptParams(const Eigen::VectorXd& opt_params)
{
	SetOptParams(opt_params, mCurrAction.mParams);
}

void cRaptorController::SetOptParams(const Eigen::VectorXd& opt_params, Eigen::VectorXd& out_params) const
{
	assert(opt_params.size() == GetNumOptParams());
	assert(gNumOptParamMasks == GetNumParams());

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

	PostProcessParams(out_params);
}

void cRaptorController::BuildFromMotion(int ctrl_params_idx, const cMotion& motion)
{
	assert(motion.IsValid());
	double dur = motion.GetDuration();
	dur -= 0.00001; // hack to prevent flipping stance at the beginning of a new cycle
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

double cRaptorController::CalcReward() const
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

		if (avg_vel < 0)
		{
			vel_reward = 0;
			stumble_reward = 0;
		}
	}
	
	double reward = 0;
	reward += vel_reward_w * vel_reward
			+ stumble_reward_w * stumble_reward;

	return reward;
}

tVector cRaptorController::GetTargetVel() const
{
	return tVector(4, 0, 0, 0);
	//return tVector(1.75, 0, 0, 0);
}

cRaptorController::eStance cRaptorController::GetStance() const
{
	return mStance;
}

double cRaptorController::GetPrevCycleTime() const
{
	return mPrevCycleTime;
}

const tVector& cRaptorController::GetPrevDistTraveled() const
{
	return mPrevDistTraveled;
}

void cRaptorController::BuildNormPose(Eigen::VectorXd& pose) const
{
	if (mChar != nullptr)
	{
		mChar->BuildPose(pose);
		eStance stance = GetStance();
		if (stance != gDefaultStance)
		{
			FlipPoseStance(pose);
		}
	}
}

bool cRaptorController::LoadControllers(const std::string& file)
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

void cRaptorController::ResetParams()
{
	cTerrainRLCharController::ResetParams();
	mState = eStateContact;
	mStance = gDefaultStance;
	
	mPrevCycleTime = 0;
	mPrevDistTraveled.setZero();
	mCurrCycleTime = 0;
	mPrevCOM.setZero();
	mPrevStumbleCount = 0;
	mCurrStumbleCount = 0;
}

bool cRaptorController::ParseControllers(const Json::Value& root)
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

		if (!root["EnableVirtualForces"].isNull())
		{
			mEnableVirtualForces = root["EnableVirtualForces"].asBool();
		}
	}

	return succ;
}

bool cRaptorController::ParseControllerFiles(const Json::Value& root)
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

bool cRaptorController::ParseActions(const Json::Value& root)
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

bool cRaptorController::ParseAction(const Json::Value& root, tBlendAction& out_action) const
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

bool cRaptorController::HasCtrlParams() const
{
	return mCtrlParams.size() > 0;
}

void cRaptorController::UpdateState(double time_step)
{
	const cRaptorController::tStateDef& state = GetCurrStateDef();
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

	if (state.mTransContact)
	{
		int swing_toe = GetSwingToe();
		bool contact = CheckContact(swing_toe);
		if (contact)
		{
			advance_state = true;
		}
	}

	if (advance_state)
	{
		eState next_state = (mFirstCycle) ? eStateContact : state.mNext;
		bool end_step = (next_state == eStateInvalid) || mFirstCycle;
		
		if (end_step)
		{
			if (!mFirstCycle)
			{
				FlipStance();
			}
			UpdateAction();
			mFirstCycle = false;
		}
		else
		{
			TransitionState(next_state);
		}
	}
}

void cRaptorController::UpdateAction()
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

void cRaptorController::UpdateRBDModel()
{
	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	mChar->BuildPose(pose);
	mChar->BuildVel(vel);

	mRBDModel->Update(pose, vel);
	cRBDUtil::BuildJacobian(*mRBDModel.get(), mJacobian);
}

void cRaptorController::UpdatePDCtrls(double time_step, Eigen::VectorXd& out_tau)
{
	mImpPDCtrl.UpdateControlForce(time_step, out_tau);
}

void cRaptorController::UpdateStumbleCounter(double time_step)
{
	bool stumbled = mChar->HasStumbled();
	if (stumbled)
	{
		mCurrStumbleCount += time_step;
	}
}

void cRaptorController::UpdateStanceHip()
{
	int stance_hip = GetStanceHip();
	int stance_toe = GetStanceToe();
	bool active = IsActiveVFEffector(stance_toe);
	mImpPDCtrl.GetPDCtrl(stance_hip).SetActive(!active);
}

void cRaptorController::ApplySwingFeedback(Eigen::VectorXd& out_tau)
{
	double cv = GetCv();
	double cd = GetCd();

	bool first_half = mState == eStateContact || mState == eStateDown;
	cd = (first_half) ? 0 : cd;
	cv = (first_half) ? cv : 0;

	tVector com = mChar->CalcCOM();
	tVector com_vel = mChar->CalcCOMVel();

	int stance_toe_id = GetStanceToe();
	int swing_hip_id = GetSwingHip();

	tVector toe_pos = mChar->GetBodyPart(stance_toe_id)->GetPos();
	tVector d = com - toe_pos;
	double d_theta = cd * d[0] + cv * com_vel[0];

	tStateParams params = GetCurrParams();
	double theta0 = params(eStateParamSwingHip);
	double theta_new = theta0 + d_theta;

	mImpPDCtrl.SetTargetTheta(swing_hip_id, theta_new);
}

void cRaptorController::ApplyStanceFeedback(Eigen::VectorXd& out_tau)
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();

	int stance_toe = GetStanceToe();
	double hip_tau = 0;

	int stance_hip_id = GetStanceHip();
	int swing_hip_id = GetSwingHip();

	const int root_joints[] = 
	{
		cSimRaptor::eJointRightHip,
		cSimRaptor::eJointLeftHip,
		//cSimRaptor::eJointSpine0,
		//cSimRaptor::eJointTail0
	};
	const int num_root_joints = sizeof(root_joints) / sizeof(root_joints[0]);

	if (IsActiveVFEffector(stance_toe))
	{
		for (int j = 0; j < num_root_joints; ++j)
		{
			int curr_id = root_joints[j];
			if (curr_id != stance_hip_id)
			{
				int param_offset = cKinTree::GetParamOffset(joint_mat, curr_id);

				// hack assuming revolute joint
				double curr_tau = out_tau(param_offset);
				hip_tau += -curr_tau;
			}
		}
		
		double target_root_pitch = GetTargetRootPitch();
		double root_pitch = GetRootPitch();
		double root_omega = mChar->GetRootAngVel()[2];

		const cPDController& stance_pd = mImpPDCtrl.GetPDCtrl(stance_hip_id);
		const double kp = stance_pd.GetKp();
		const double kd = stance_pd.GetKd();

		double root_tau = kp * (target_root_pitch - root_pitch) + kd * (-root_omega);
		hip_tau += -root_tau;

		int stance_hip_offset = cKinTree::GetParamOffset(joint_mat, stance_hip_id);
		int stance_hip_size = cKinTree::GetParamSize(joint_mat, stance_hip_id);
		auto stance_tau = out_tau.segment(stance_hip_offset, stance_hip_size);
		stance_tau += hip_tau * Eigen::VectorXd::Ones(stance_hip_size);
	}
}

void cRaptorController::ApplyGravityCompensation(Eigen::VectorXd& out_tau)
{
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

		Eigen::VectorXd W(3);
		W(0) = 0.0001;
		W(1) = 0.0001;
		W(2) = 1;

		Eigen::MatrixXd AtA = A.transpose() * W.asDiagonal() * A;
		Eigen::VectorXd Atb = A.transpose() * W.asDiagonal() * b;
		AtA.diagonal() += lambda * Eigen::VectorXd::Ones(basis_cols);

		Eigen::VectorXd x = (AtA).householderQr().solve(Atb);
		Eigen::VectorXd tau_contact = contact_basis * x;

		tau_g -= tau_contact;
		out_tau += tau_g;
	}
}

void cRaptorController::ApplyVirtualForces(Eigen::VectorXd& out_tau)
{
	for (int e = 0; e < gNumEndEffectors; ++e)
	{
		cSimRaptor::eJoint joint_id = gEndEffectors[e];
		bool active_effector = IsActiveVFEffector(joint_id);
		
		int stance_hip_id = GetStanceHip();
		int swing_hip_id = GetSwingHip();
		int root_id = mChar->GetRootID();

		if (active_effector)
		{
			tVector vf = -GetEffectorVF();
			
			tVector pos = GetEndEffectorContactPos(joint_id);
			cSpAlg::tSpTrans joint_world_trans = cSpAlg::BuildTrans(-pos);

			cSpAlg::tSpVec sp_force = cSpAlg::BuildSV(vf);
			sp_force = cSpAlg::ApplyTransF(joint_world_trans, sp_force);

			const Eigen::MatrixXd& joint_mat = mRBDModel->GetJointMat();
			int curr_id = joint_id;
			while (curr_id != root_id)
			{
				assert(curr_id != cKinTree::gInvalidJointID);
				int offset = cKinTree::GetParamOffset(joint_mat, curr_id);
				int size = cKinTree::GetParamSize(joint_mat, curr_id);
				const auto curr_J = mJacobian.block(0, offset, cSpAlg::gSpVecSize, size);

				out_tau.segment(offset, size) += curr_J.transpose() * sp_force;

				if (curr_id == stance_hip_id)
				{
					int swing_offset = cKinTree::GetParamOffset(joint_mat, swing_hip_id);
					int swing_size = cKinTree::GetParamSize(joint_mat, swing_hip_id);

					out_tau.segment(swing_offset, swing_size) += -(curr_J.transpose() * sp_force);
				}

				curr_id = mRBDModel->GetParent(curr_id);
			}
		}
	}
}

const cRaptorController::tStateDef& cRaptorController::GetCurrStateDef() const
{
	return gStateDefs[mState];
}

cRaptorController::tStateParams cRaptorController::GetCurrParams() const
{
	tStateParams params = mCurrAction.mParams.segment(eMiscParamMax + mState * eStateParamMax, eStateParamMax);
	return params;
}

double cRaptorController::GetTargetRootPitch() const
{
	tStateParams params = GetCurrParams();
	double pitch = params(eStateParamRootPitch);
	return pitch;
}

double cRaptorController::GetRootPitch() const
{
	tVector axis = tVector::Zero();
	double theta = 0;
	mChar->GetRootRotation(axis, theta);

	const tVector& ref_axis = tVector(0, 0, 1, 0);
	if (axis.dot(ref_axis) < 0)
	{
		theta = -theta;
	}
	return theta;
}

void cRaptorController::SetStateParams(const tStateParams& params)
{
#if defined(ENABLE_SPINE_CURVE)
	double spine_theta = params(eStateParamSpineCurve);
	for (int i = 0; i < gNumSpineJoints; ++i)
	{
		cSimRaptor::eJoint joint = gSpineJoints[i];
		mImpPDCtrl.SetTargetTheta(joint, spine_theta);
	}
#endif

	mImpPDCtrl.SetTargetTheta(GetStanceHip(), params(eStateParamStanceHip));
	mImpPDCtrl.SetTargetTheta(GetStanceKnee(), params(eStateParamStanceKnee));
	mImpPDCtrl.SetTargetTheta(GetStanceAnkle(), params(eStateParamStanceAnkle));
	mImpPDCtrl.SetTargetTheta(GetSwingHip(), params(eStateParamSwingHip));
	mImpPDCtrl.SetTargetTheta(GetSwingKnee(), params(eStateParamSwingKnee));
	mImpPDCtrl.SetTargetTheta(GetSwingAnkle(), params(eStateParamSwingAnkle));
}

void cRaptorController::SetupPassiveMode()
{
	tStateParams params;
	params.setZero();
	SetStateParams(params);
}

double cRaptorController::GetTransTime() const
{
	return mCurrAction.mParams[eMiscParamTransTime];
}

double cRaptorController::GetCv() const
{
	return mCurrAction.mParams[eMiscParamCv];
}

double cRaptorController::GetCd() const
{
	return mCurrAction.mParams[eMiscParamCd];
}

bool cRaptorController::CheckContact(int joint_id) const
{
	assert(joint_id != cSimRaptor::eJointInvalid);
	const auto& body_part = mChar->GetBodyPart(joint_id);
	bool contact = body_part->IsInContact();
	return contact;
}

bool cRaptorController::IsActiveVFEffector(int joint_id) const
{
	int state = GetState();
	bool valid_effector = (joint_id == GetStanceToe()) && (state == eStateContact || state == eStateDown);
	bool active_effector = valid_effector && CheckContact(joint_id);
	return active_effector;
}

tVector cRaptorController::GetEffectorVF() const
{
	return tVector(mCurrAction.mParams[eMiscParamForceX], mCurrAction.mParams[eMiscParamForceY], 0, 0);
}

Eigen::MatrixXd cRaptorController::BuildContactBasis(const Eigen::VectorXd& pose, bool& out_has_support) const
{
	const int num_basis = 2;
	int rows = static_cast<int>(pose.size());
	const int cols = gNumEndEffectors * num_basis;

	const double cone_theta = M_PI * 0.1;
	double s = std::sin(cone_theta);
	double c = std::cos(cone_theta);

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
		cSimRaptor::eJoint joint_id = gEndEffectors[e];
		bool valid_support = IsActiveVFEffector(joint_id);
		//bool valid_support = CheckContact(joint_id);

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

const std::string& cRaptorController::GetStateName(eState state) const
{
	return gStateDefs[state].mName;
}

const std::string& cRaptorController::GetStateParamName(eStateParam param) const
{
	return gStateParamNames[param];
}

void cRaptorController::GetOptParams(const Eigen::VectorXd& ctrl_params, Eigen::VectorXd& out_opt_params) const
{
	int num_params = GetNumParams();
	int num_opt_params = GetNumOptParams();
	assert(ctrl_params.size() == num_params);
	assert(gNumOptParamMasks == num_params);

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

std::string cRaptorController::BuildOptParamsJson(const Eigen::VectorXd& opt_params) const
{
	assert(opt_params.size() == GetNumOptParams());
	Eigen::VectorXd param_buffer = mCurrAction.mParams;
	SetOptParams(opt_params, param_buffer);

	std::string json = "";
	int idx = 0;

	json += "\"" + gMiscParamsKey + "\": \n{\n";
	for (int i = 0; i < eMiscParamMax; ++i)
	{
		if (i != 0)
		{
			json += ",\n";
		}
		json += "\"" + gMiscParamsNames[i] +  "\": " + std::to_string(param_buffer[idx++]);
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
			json += "\"" + gStateParamNames[j] + "\": " + std::to_string(param_buffer[idx++]);
		}
		json += "\n}";
	}
	json += "\n}\n";

	json = "{" + json + "}";
	return json;
}

void cRaptorController::BuildStateParamsFromPose(const Eigen::VectorXd& pose, tStateParams& out_params)
{
	const cSimRaptor::eJoint ctrl_joints[] =
	{
		cSimRaptor::eJointRightHip,
		cSimRaptor::eJointRightKnee,
		cSimRaptor::eJointRightAnkle,
		cSimRaptor::eJointLeftHip,
		cSimRaptor::eJointLeftKnee,
		cSimRaptor::eJointLeftAnkle,
	};
	const int num_ctrl_joints = sizeof(ctrl_joints) / sizeof(ctrl_joints[0]);

	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();
	
	double root_theta = cKinTree::GetRootTheta(joint_mat, pose);
	out_params(eStateParamRootPitch) = root_theta;
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
		out_params(eStateParamStanceHip + i) = tar_theta;
	}
}

double cRaptorController::CalcSpineCurve(const Eigen::VectorXd& pose) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();

	double curve = 0;
	for (int i = 0; i < gNumSpineJoints; ++i)
	{
		cSimRaptor::eJoint joint = gSpineJoints[i];
		double theta = cKinTree::GetJointTheta(joint_mat, pose, joint);
		curve += theta;
	}

	curve /= gNumSpineJoints;
	return curve;
}

const Eigen::VectorXd& cRaptorController::GetCtrlParams(int ctrl_id) const
{
	return mCtrlParams[ctrl_id];
}

bool cRaptorController::IsCurrActionCyclic() const
{
	const tBlendAction& action = mActions[mCurrAction.mID];
	return action.mCyclic;
}

void cRaptorController::ApplyAction(int action_id)
{
	cTerrainRLCharController::ApplyAction(action_id);
}

void cRaptorController::ApplyAction(const tAction& action)
{
	cTerrainRLCharController::ApplyAction(action);
	TransitionState(eStateContact);
}

void cRaptorController::NewCycleUpdate()
{
	cTerrainRLCharController::NewCycleUpdate();
	mPrevCycleTime = mCurrCycleTime;
	mCurrCycleTime = 0;
	mPrevStumbleCount = mCurrStumbleCount;
	mCurrStumbleCount = 0;
	RecordDistTraveled();
	mPrevCOM = mChar->CalcCOM();
}

void cRaptorController::BlendCtrlParams(const tBlendAction& action, Eigen::VectorXd& out_params) const
{
	const Eigen::VectorXd& param0 = GetCtrlParams(action.mParamIdx0);
	const Eigen::VectorXd& param1 = GetCtrlParams(action.mParamIdx1);
	double blend = action.mBlend;
	out_params = (1 - blend) * param0 + blend * param1;
}

void cRaptorController::PostProcessParams(Eigen::VectorXd& out_params) const
{
	out_params[eMiscParamTransTime] = std::abs(out_params[eMiscParamTransTime]);
	out_params[eMiscParamCv] = std::abs(out_params[eMiscParamCv]);
	out_params[eMiscParamCd] = std::abs(out_params[eMiscParamCd]);
}

bool cRaptorController::IsOptParam(int param_idx) const
{
	assert(param_idx >= 0 && param_idx < gNumOptParamMasks);
	return gOptParamsMasks[param_idx];
}

void cRaptorController::BuildPoliStatePose(Eigen::VectorXd& out_pose) const
{
	cTerrainRLCharController::BuildPoliStatePose(out_pose);
	eStance stance = GetStance();
	if (stance != gDefaultStance)
	{
		FlipPoliPoseStance(out_pose);
	}
}

void cRaptorController::BuildPoliStateVel(Eigen::VectorXd& out_vel) const
{
	cTerrainRLCharController::BuildPoliStateVel(out_vel);
	eStance stance = GetStance();
	if (stance != gDefaultStance)
	{
		FlipPoliPoseStance(out_vel);
	}
}

void cRaptorController::FlipStance()
{
	SetStance((mStance == eStanceRight) ? eStanceLeft : eStanceRight);
}

void cRaptorController::SetStance(eStance stance)
{
	mStance = stance;
	mImpPDCtrl.GetPDCtrl(GetStanceHip()).SetActive(false);
	mImpPDCtrl.GetPDCtrl(GetSwingHip()).SetActive(true);

	TransitionState(GetState(), GetPhase());
}

void cRaptorController::FlipPoseStance(Eigen::VectorXd& out_pose) const
{
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();

	int num_dofs = mChar->GetNumDof();
	int last_body_joint = cSimRaptor::eJointTail4;
	int start_idx = cKinTree::GetParamOffset(joint_mat, last_body_joint);
	start_idx += cKinTree::GetParamSize(joint_mat, last_body_joint);

	int num_leg_params = (num_dofs - start_idx) / 2;
	for (int i = 0; i < num_leg_params; ++i)
	{
		int idx0 = start_idx + i;
		int idx1 = start_idx + num_leg_params + i;
		double val0 = out_pose(idx0);
		double val1 = out_pose(idx1);
		out_pose(idx0) = val1;
		out_pose(idx1) = val0;
	}
}

void cRaptorController::FlipPoliPoseStance(Eigen::VectorXd& out_pose) const
{
	// hack
	// assume that the leg params are all packed at the end of the vector!
	const Eigen::MatrixXd& joint_mat = mChar->GetJointMat();

	int num_params = static_cast<int>(out_pose.size());
	int num_leg_joints = GetNumJointsPerLeg();
	int num_leg_params = num_leg_joints * gPosDim;

	for (int i = 0; i < num_leg_params; ++i)
	{
		int idx0 = num_params - i - 1;
		int idx1 = num_params - num_leg_params - i - 1;
		double val0 = out_pose(idx0);
		double val1 = out_pose(idx1);
		out_pose(idx0) = val1;
		out_pose(idx1) = val0;
	}
}

int cRaptorController::GetNumJointsPerLeg() const
{
	int hip_id = GetStanceHip();
	int toe_id = GetStanceToe();
	int num_joints = std::abs(toe_id - hip_id) + 1;
	return num_joints;
}

int cRaptorController::GetSwingHip() const
{
	return (GetStance() == eStanceRight) ? cSimRaptor::eJointLeftHip : cSimRaptor::eJointRightHip;
}

int cRaptorController::GetSwingKnee() const
{
	return (GetStance() == eStanceRight) ? cSimRaptor::eJointLeftKnee : cSimRaptor::eJointRightKnee;
}

int cRaptorController::GetSwingAnkle() const
{
	return (GetStance() == eStanceRight) ? cSimRaptor::eJointLeftAnkle : cSimRaptor::eJointRightAnkle;
}

int cRaptorController::GetSwingToe() const
{
	return (GetStance() == eStanceRight) ? cSimRaptor::eJointLeftToe : cSimRaptor::eJointRightToe;
}

int cRaptorController::GetStanceHip() const
{
	return (GetStance() == eStanceRight) ? cSimRaptor::eJointRightHip : cSimRaptor::eJointLeftHip;
}

int cRaptorController::GetStanceKnee() const
{
	return (GetStance() == eStanceRight) ? cSimRaptor::eJointRightKnee : cSimRaptor::eJointLeftKnee;
}

int cRaptorController::GetStanceAnkle() const
{
	return (GetStance() == eStanceRight) ? cSimRaptor::eJointRightAnkle : cSimRaptor::eJointLeftAnkle;
}

int cRaptorController::GetStanceToe() const
{
	return (GetStance() == eStanceRight) ? cSimRaptor::eJointRightToe : cSimRaptor::eJointLeftToe;
}

tVector cRaptorController::GetEndEffectorContactPos(int joint_id) const
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

void cRaptorController::RecordDistTraveled()
{
	tVector com = mChar->CalcCOM();
	mPrevDistTraveled = com - mPrevCOM;
}

int cRaptorController::PopCommand()
{
	int cmd = mCommands.top();
	mCommands.pop();
	return cmd;
}

bool cRaptorController::HasCommands() const
{
	return !mCommands.empty();
}

void cRaptorController::ClearCommands()
{
	while (!mCommands.empty())
	{
		mCommands.pop();
	}
}

void cRaptorController::ProcessCommand(tAction& out_action)
{
	int a_id = PopCommand();
	BuildBaseAction(a_id, out_action);
}

void cRaptorController::BuildBaseAction(int action_id, tAction& out_action) const
{
	const tBlendAction& blend = mActions[action_id];
	out_action.mID = blend.mID;
	BlendCtrlParams(blend, out_action.mParams);
}
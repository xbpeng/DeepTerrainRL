#include "TerrainRLCharController.h"
#include "sim/SimCharacter.h"
#include <iostream>
#include <ctime>
#include <json/json.h>

#define ENABLE_MAX_COORD_POSE

const int cTerrainRLCharController::gPosDim = 2; // 2D position coords
const int gNumGroundSamples = 200;
const double gViewMin = -0.5;
const double gDefaultViewDist = 10;

#if defined(ENABLE_DEBUG_VISUALIZATION)
const int gPoliValLogSize = 50;
#endif // ENABLE_DEBUG_VISUALIZATION

cTerrainRLCharController::cTerrainRLCharController() : cNNController()
{
	mCurrAction.mID = gInvalidIdx;
	mCurrAction.mParams.resize(0);
	SetViewDist(gDefaultViewDist);
}

cTerrainRLCharController::~cTerrainRLCharController()
{
}

void cTerrainRLCharController::Init(cSimCharacter* character)
{
	// param_file should contain parameters for the pd controllers
	cNNController::Init(character);
	mCurrAction.mParams.resize(GetNumParams());
	ResetParams();
	
	mGroundSamples = Eigen::VectorXd::Zero(gNumGroundSamples);
	mPoliState = Eigen::VectorXd::Zero(GetPoliStateSize());

	mValid = true;

#if defined(ENABLE_DEBUG_VISUALIZATION)
	mPoliValLog.Reserve(gPoliValLogSize);
	mPoliValLog.Clear();
#endif // ENABLE_DEBUG_VISUALIZATION
}

void cTerrainRLCharController::Reset()
{
	ApplyAction(GetDefaultAction());
	cNNController::Reset();
	ResetParams();

	mGroundSamples = Eigen::VectorXd::Zero(gNumGroundSamples);

#if defined(ENABLE_DEBUG_VISUALIZATION)
	mPoliValLog.Clear();
#endif // ENABLE_DEBUG_VISUALIZATION
}

void cTerrainRLCharController::Clear()
{
	cNNController::Clear();
	ResetParams();
	mCurrAction.mID = gInvalidIdx;
}

void cTerrainRLCharController::Update(double time_step)
{
	cNNController::Update(time_step);
}

void cTerrainRLCharController::SetGround(std::shared_ptr<cGround> ground)
{
	mGround = ground;
}

int cTerrainRLCharController::GetCurrActionID() const
{
	return mCurrAction.mID;
}

int cTerrainRLCharController::GetNumGroundSamples() const
{
	return static_cast<int>(mGroundSamples.size());
}

tVector cTerrainRLCharController::GetGroundSample(int s) const
{
	tVector pos = CalcGroundSamplePos(s);
	double h = mGroundSamples[s];
	pos[1] += h;
	return pos;
}

tVector cTerrainRLCharController::GetGroundSampleOrigin() const
{
	return mGroundSampleOrigin;
}

int cTerrainRLCharController::GetPoliStateSize() const
{
	int state_size = 0;
	for (int i = 0; i < ePoliStateMax; ++i)
	{
		state_size += GetPoliStateSize(static_cast<ePoliState>(i));
	}
	return state_size;
}

bool cTerrainRLCharController::IsOffPolicy() const
{
	return mIsOffPolicy;
}

int cTerrainRLCharController::GetPoliActionSize() const
{
	return GetNumActions();
}

void cTerrainRLCharController::RecordPoliState(Eigen::VectorXd& out_state) const
{
	out_state = mPoliState;
}

void cTerrainRLCharController::ResetParams()
{
	mPhase = 0;
	mFirstCycle = true;
	mIsOffPolicy = false;
	mGroundSampleOrigin.setZero();
}

void cTerrainRLCharController::ApplyAction(int action_id)
{
	BuildBaseAction(action_id, mCurrAction);
	ApplyAction(mCurrAction);
}

void cTerrainRLCharController::ApplyAction(const tAction& action)
{
	mCurrAction = action;
	PostProcessParams(mCurrAction.mParams);

	NewCycleUpdate();
	SetParams(mCurrAction.mParams);
}

void cTerrainRLCharController::NewCycleUpdate()
{
}

void cTerrainRLCharController::SetParams(const Eigen::VectorXd& params)
{
	assert(params.size() == GetNumParams());
	mCurrAction.mParams = params;
	PostProcessParams(mCurrAction.mParams);
}

bool cTerrainRLCharController::IsOptParam(int param_idx) const
{
	return true;
}

void cTerrainRLCharController::PostProcessParams(Eigen::VectorXd& out_params) const
{
}

void cTerrainRLCharController::UpdateGroundSampleOrigin()
{
	mGroundSampleOrigin = mChar->GetRootPos();
	double ground_h = 0;
	if (mGround != nullptr)
	{
		ground_h = mGround->SampleHeight(mGroundSampleOrigin);
	}
	mGroundSampleOrigin[1] = ground_h;
}

void cTerrainRLCharController::ParseGround()
{
	UpdateGroundSampleOrigin();
	if (HasGround())
	{
		SampleGround(mGroundSamples);
	}
	else
	{
		mGroundSamples.setZero();
	}
}

bool cTerrainRLCharController::HasGround() const
{
	return mGround != nullptr;
}

void cTerrainRLCharController::SampleGround(Eigen::VectorXd& out_samples) const
{
	for (int i = 0; i < gNumGroundSamples; ++i)
	{
		tVector sample_pos = CalcGroundSamplePos(i);
		double h = mGround->SampleHeight(sample_pos);
		h -= mGroundSampleOrigin[1];
		out_samples[i] = h;
	}
}

tVector cTerrainRLCharController::CalcGroundSamplePos(int s) const
{
	double view_dist = GetViewDist();
	double dist = ((view_dist - gViewMin) * s) / (gNumGroundSamples - 1) + gViewMin;
	return tVector(dist, 0, 0, 0) + mGroundSampleOrigin;
}

void cTerrainRLCharController::BuildPoliState(Eigen::VectorXd& out_state) const
{
	int state_size = GetPoliStateSize();
	out_state.resize(state_size);

	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	BuildPoliStatePose(pose);
	BuildPoliStateVel(vel);

	int ground_offset = GetPoliStateOffset(ePoliStateGround);
	int ground_size = GetPoliStateSize(ePoliStateGround);
	int pose_offset = GetPoliStateOffset(ePoliStatePose);
	int pose_size = GetPoliStateSize(ePoliStatePose);
	int vel_offset = GetPoliStateOffset(ePoliStateVel);
	int vel_size = GetPoliStateSize(ePoliStateVel);

	out_state.segment(ground_offset, ground_size) = mGroundSamples;
	out_state.segment(pose_offset, pose_size) = pose;
	out_state.segment(vel_offset, vel_size) = vel;
}

void cTerrainRLCharController::BuildPoliStatePose(Eigen::VectorXd& out_pose) const
{
	tVector root_pos = mChar->GetRootPos();
	double ground_h = mGround->SampleHeight(root_pos);
	tVector root_pos_rel = root_pos;
	root_pos_rel[1] -= ground_h;

#if defined(ENABLE_MAX_COORD_POSE)
	out_pose.resize(GetPoliStateSize(ePoliStatePose));
	out_pose[0] = root_pos_rel[1];
	int num_parts = mChar->GetNumBodyParts();

	int idx = 1;
	for (int i = 1; i < num_parts; ++i)
	{
		const auto& curr_part = mChar->GetBodyPart(i);
		tVector curr_pos = curr_part->GetPos();
		curr_pos -= root_pos;

		out_pose.segment(idx, gPosDim) = curr_pos.segment(0, gPosDim);
		idx += gPosDim;
	}
#else
	Eigen::VectorXd pose;
	mChar->BuildPose(pose);
	cKinTree::SetRootPos(mChar->GetJointMat(), root_pos_rel, pose);
	out_pose = pose.segment(1, pose.size() - 1);
#endif
}

void cTerrainRLCharController::BuildPoliStateVel(Eigen::VectorXd& out_vel) const
{
#if defined(ENABLE_MAX_COORD_POSE)
	out_vel.resize(GetPoliStateSize(ePoliStateVel));
	int num_parts = mChar->GetNumBodyParts();

	int idx = 0;
	for (int i = 0; i < num_parts; ++i)
	{
		const auto& curr_part = mChar->GetBodyPart(i);
		tVector curr_vel = curr_part->GetLinearVelocity();

		out_vel.segment(idx, gPosDim) = curr_vel.segment(0, gPosDim);
		idx += gPosDim;
	}
#else
	mChar->BuildVel(out_vel);
#endif
}

int cTerrainRLCharController::GetPoliStateOffset(ePoliState params) const
{
	int offset = 0;
	switch (params)
	{
	case ePoliStateGround:
		offset = 0;
		break;
	case ePoliStatePose:
		offset = GetPoliStateSize(ePoliStateGround);
		break;
	case ePoliStateVel:
		offset = GetPoliStateSize(ePoliStateGround) + GetPoliStateSize(ePoliStatePose);
		break;
	default:
		assert(false); // unsupported poli state param
		break;
	}
	return offset;
}

int cTerrainRLCharController::GetPoliStateSize(ePoliState params) const
{
	int size = 0;
	switch (params)
	{
	case ePoliStateGround:
		size = GetNumGroundSamples();
		break;
#if defined(ENABLE_MAX_COORD_POSE)
	case ePoliStatePose:
		size = mChar->GetNumBodyParts() * gPosDim - 1; // -1 for root x
#if defined(ENABLE_POLI_STATE_THETA)
		size += mChar->GetNumBodyParts();
#endif
		break;
	case ePoliStateVel:
		size = mChar->GetNumBodyParts() * gPosDim;
#if defined(ENABLE_POLI_STATE_THETA)
		size += mChar->GetNumBodyParts();
#endif
		break;
#else
	case ePoliStatePose:
		size = mChar->GetNumDof() - 1; // -1 for root x
		break;
	case ePoliStateVel:
		size = mChar->GetNumDof();
		break;
#endif
	default:
		assert(false); // unsupported poli state param
		break;
	}
	return size;
}

void cTerrainRLCharController::BuildDefaultAction(tAction& out_action) const
{
	BuildBaseAction(GetDefaultAction(), out_action);
}

void cTerrainRLCharController::BuildRandBaseAction(tAction& out_action) const
{
	int num_actions = GetNumActions();
	int a = cMathUtil::RandInt(0, num_actions);
	BuildBaseAction(a, out_action);

#if defined (ENABLE_DEBUG_PRINT)
	printf("rand action: %i\n", a);
#endif
}

#if defined(ENABLE_DEBUG_VISUALIZATION)
const cCircularBuffer<double>& cTerrainRLCharController::GetPoliValLog() const
{
	return mPoliValLog;
}

void cTerrainRLCharController::GetVisCharacterFeatures(Eigen::VectorXd& out_features) const
{
	int pose_offset = GetPoliStateOffset(ePoliStatePose);
	int pose_size = GetPoliStateSize(ePoliStatePose);
	int vel_offset = GetPoliStateOffset(ePoliStateVel);
	int vel_size = GetPoliStateSize(ePoliStateVel);

	Eigen::VectorXd norm_features;
	RecordPoliState(norm_features);
	if (HasNet())
	{
		mNet.NormalizeInput(norm_features);
	}

	out_features.resize(pose_size + vel_size);
	out_features.segment(0, pose_size) = norm_features.segment(pose_offset, pose_size);
	out_features.segment(pose_size, vel_size) = norm_features.segment(vel_offset, vel_size);
}

void cTerrainRLCharController::GetVisTerrainFeatures(Eigen::VectorXd& out_features) const
{
	int ground_offset = GetPoliStateOffset(ePoliStateGround);
	int ground_size = GetPoliStateSize(ePoliStateGround);

	Eigen::VectorXd norm_features;
	RecordPoliState(norm_features);
	if (HasNet())
	{
		mNet.NormalizeInput(norm_features);
	}

	out_features = norm_features.segment(ground_offset, ground_size);
}

void cTerrainRLCharController::GetVisActionFeatures(Eigen::VectorXd& out_features) const
{
	RecordPoliAction(out_features);
	if (HasNet())
	{
		mNet.NormalizeOutput(out_features);
	}
}

void cTerrainRLCharController::GetVisActionValues(Eigen::VectorXd& out_features) const
{
	out_features = mVisNNOutput;
}
#endif // ENABLE_DEBUG_VISUALIZATION

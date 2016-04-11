#include "CharController.h"
#include "SimCharacter.h"

cCharController::cCharController()
{
	mState = 0;
	mPhase = 0;

	mEnableExp = false;
	mExpRate = 0.2;
	mExpTemp = 1;
	mExpBaseActionRate = 0;
}

cCharController::~cCharController()
{
}

void cCharController::Reset()
{
	cController::Reset();
	TransitionState(0, 0);
}

void cCharController::Update(double time_step)
{
	cController::Update(time_step);
}

int cCharController::GetState() const
{
	return mState;
}

double cCharController::GetPhase() const
{
	return mPhase;
}

void cCharController::SetPhase(double phase)
{
	mPhase = phase;
}

int cCharController::GetNumStates() const
{
	return 1;
}

double cCharController::CalcNormPhase() const
{
	int state = GetState();
	double phase = GetPhase();
	phase = cMathUtil::Clamp(phase, 0.0, 1.0);

	int num_states = GetNumStates();
	double norm_phase = (state + phase) / num_states;
	norm_phase = cMathUtil::Clamp(norm_phase, 0.0, 1.0);
	return norm_phase;
}

void cCharController::TransitionState(int state)
{
	TransitionState(state, 0);
}

void cCharController::TransitionState(int state, double phase)
{
	mState = state;
	mPhase = phase;
}

bool cCharController::IsNewCycle() const
{
	return mState == 0 && mPhase == 0;
}

void cCharController::CommandAction(int action_id)
{
}

void cCharController::CommandRandAction()
{
}

int cCharController::GetDefaultAction() const
{
	return gInvalidIdx;
}

void cCharController::SetDefaultAction(int action_id)
{
}

int cCharController::GetNumActions() const
{
	return 0;
}

int cCharController::GetCurrActionID() const
{
	return 0;
}

void cCharController::EnableExp(bool enable)
{
	mEnableExp = enable;
}

void cCharController::SetExpRate(double rate)
{
	mExpRate = rate;
}

void cCharController::SetExpTemp(double temp)
{
	mExpTemp = temp;
}

void cCharController::SetExpBaseActionRate(double rate)
{
	mExpBaseActionRate = rate;
}

bool cCharController::EnabledExplore() const
{
	return mEnableExp;
}

double cCharController::GetExpRate() const
{
	return mExpRate;
}

double cCharController::GetExpTemp() const
{
	return mExpTemp;
}

double cCharController::GetExpBaseActionRate() const
{
	return mExpBaseActionRate;
}

double cCharController::GetViewDist() const
{
	return mViewDist;
}

void cCharController::SetViewDist(double dist)
{
	mViewDist = dist;
}

void cCharController::BuildNormPose(Eigen::VectorXd& pose) const
{
	if (mChar != nullptr)
	{
		mChar->BuildPose(pose);
	}
}

void cCharController::BuildFromMotion(int ctrl_params_idx, const cMotion& motion)
{
}

void cCharController::BuildCtrlOptParams(int ctrl_params_idx, Eigen::VectorXd& out_params) const
{
}

void cCharController::SetCtrlOptParams(int ctrl_params_idx, const Eigen::VectorXd& params)
{
}

void cCharController::BuildActionOptParams(int action_id, Eigen::VectorXd& out_params) const
{
}

int cCharController::GetNumGroundSamples() const
{
	return 0;
}

tVector cCharController::GetGroundSample(int s) const
{
	return tVector::Zero();
}

tVector cCharController::GetGroundSampleOrigin() const
{
	return tVector::Zero();
}

#if defined(ENABLE_DEBUG_VISUALIZATION)
void cCharController::GetVisCharacterFeatures(Eigen::VectorXd& out_features) const
{
	out_features = Eigen::VectorXd();
}

void cCharController::GetVisTerrainFeatures(Eigen::VectorXd& out_features) const
{
	out_features = Eigen::VectorXd();
}

void cCharController::GetVisActionFeatures(Eigen::VectorXd& out_features) const
{
	out_features = Eigen::VectorXd();
}

void cCharController::GetVisActionValues(Eigen::VectorXd& out_vals) const
{
	out_vals = Eigen::VectorXd();
}
#endif
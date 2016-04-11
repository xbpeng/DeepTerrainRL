#pragma once

#include "Controller.h"
#include "anim/Motion.h"

class cCharController : public cController
{
public:
	virtual ~cCharController();

	virtual void Reset();
	virtual void Update(double time_step);
	virtual int GetState() const;
	virtual double GetPhase() const;
	virtual void SetPhase(double phase);
	virtual int GetNumStates() const;
	virtual double CalcNormPhase() const;

	virtual void TransitionState(int state);
	virtual void TransitionState(int state, double phase);
	virtual bool IsNewCycle() const;

	virtual void CommandAction(int action_id);
	virtual void CommandRandAction();
	virtual int GetDefaultAction() const;
	virtual void SetDefaultAction(int action_id);
	virtual int GetNumActions() const;
	virtual int GetCurrActionID() const;

	virtual void EnableExp(bool enable);
	virtual void SetExpRate(double rate);
	virtual void SetExpTemp(double temp);
	virtual void SetExpBaseActionRate(double rate);
	virtual bool EnabledExplore() const;
	virtual double GetExpRate() const;
	virtual double GetExpTemp() const;
	virtual double GetExpBaseActionRate() const;

	virtual double GetViewDist() const;
	virtual void SetViewDist(double dist);

	virtual void BuildNormPose(Eigen::VectorXd& pose) const;

	virtual void BuildFromMotion(int ctrl_params_idx, const cMotion& motion);
	virtual void BuildCtrlOptParams(int ctrl_params_idx, Eigen::VectorXd& out_params) const;
	virtual void SetCtrlOptParams(int ctrl_params_idx, const Eigen::VectorXd& params);
	virtual void BuildActionOptParams(int action_id, Eigen::VectorXd& out_params) const;

	virtual int GetNumGroundSamples() const;
	virtual tVector GetGroundSample(int s) const;
	virtual tVector GetGroundSampleOrigin() const;

protected:
	int mState;
	double mPhase;

	bool mEnableExp;
	double mExpRate;
	double mExpTemp;
	double mExpBaseActionRate;

	double mViewDist;

	cCharController();

#if defined(ENABLE_DEBUG_VISUALIZATION)
public:
	virtual void GetVisCharacterFeatures(Eigen::VectorXd& out_features) const;
	virtual void GetVisTerrainFeatures(Eigen::VectorXd& out_features) const;
	virtual void GetVisActionFeatures(Eigen::VectorXd& out_features) const;
	virtual void GetVisActionValues(Eigen::VectorXd& out_vals) const;
#endif
};

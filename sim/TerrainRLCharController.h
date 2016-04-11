#pragma once

#include "sim/Ground.h"
#include "sim/NNController.h"
#include "util/CircularBuffer.h"

class cTerrainRLCharController : public cNNController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	virtual ~cTerrainRLCharController();

	virtual void Init(cSimCharacter* character);
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_step);

	virtual void SetGround(std::shared_ptr<cGround> ground);
	
	virtual int GetNumActions() const = 0;
	virtual int GetCurrActionID() const;
	virtual int GetNumParams() const = 0;

	virtual int GetNumGroundSamples() const;
	virtual tVector GetGroundSample(int s) const;
	virtual tVector GetGroundSampleOrigin() const;

	virtual bool IsOffPolicy() const;
	virtual int GetPoliStateSize() const;
	virtual int GetPoliActionSize() const;
	virtual void RecordPoliState(Eigen::VectorXd& out_state) const;
	virtual void RecordPoliAction(Eigen::VectorXd& out_action) const = 0;

	virtual void BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const = 0;

protected:
	enum ePoliState
	{
		ePoliStateGround,
		ePoliStatePose,
		ePoliStateVel,
		ePoliStateMax
	};

	struct tAction
	{
		int mID;
		Eigen::VectorXd mParams;
	};
	
	static const int gPosDim;

	bool mFirstCycle;
	bool mIsOffPolicy;
	tAction mCurrAction;
	Eigen::VectorXd mPoliState;

	tVector mGroundSampleOrigin;
	std::shared_ptr<cGround> mGround;
	Eigen::VectorXd mGroundSamples;

	cTerrainRLCharController();

	virtual void ResetParams();

	virtual void ApplyAction(int action_id);
	virtual void ApplyAction(const tAction& action);
	virtual void NewCycleUpdate();
	virtual void PostProcessParams(Eigen::VectorXd& out_params) const;
	virtual void SetParams(const Eigen::VectorXd& params);
	virtual bool IsOptParam(int param_idx) const;

	virtual void UpdateGroundSampleOrigin();
	virtual void ParseGround();
	virtual bool HasGround() const;
	virtual void SampleGround(Eigen::VectorXd& out_samples) const;
	virtual tVector CalcGroundSamplePos(int s) const;

	virtual void BuildPoliState(Eigen::VectorXd& out_state) const;
	virtual void BuildPoliStatePose(Eigen::VectorXd& out_pose) const;
	virtual void BuildPoliStateVel(Eigen::VectorXd& out_vel) const;
	virtual int GetPoliStateOffset(ePoliState params) const;
	virtual int GetPoliStateSize(ePoliState params) const;

	virtual void DecideAction(tAction& out_action) = 0;
	virtual void ExploitPolicy(tAction& out_action) = 0;
	virtual void ExploreAction(tAction& out_action) = 0;

	virtual void BuildDefaultAction(tAction& out_action) const;
	virtual void BuildBaseAction(int action_id, tAction& out_action) const = 0;
	virtual void BuildRandBaseAction(tAction& out_action) const;

#if defined(ENABLE_DEBUG_VISUALIZATION)
public:
	const cCircularBuffer<double>& GetPoliValLog() const;
	virtual void GetVisCharacterFeatures(Eigen::VectorXd& out_features) const;
	virtual void GetVisTerrainFeatures(Eigen::VectorXd& out_features) const;
	virtual void GetVisActionFeatures(Eigen::VectorXd& out_features) const;
	virtual void GetVisActionValues(Eigen::VectorXd& out_vals) const;

protected:
	cCircularBuffer<double> mPoliValLog;
	Eigen::VectorXd mVisNNOutput;
#endif // ENABLE_DEBUG_VISUALIZATION
};
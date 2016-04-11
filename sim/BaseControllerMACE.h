#pragma once

#include "sim/TerrainRLCharController.h"
#include "learning/MACETrainer.h"

//#define DISABLE_INIT_ACTOR_BIAS

class cBaseControllerMACE : public virtual cTerrainRLCharController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cBaseControllerMACE();
	virtual ~cBaseControllerMACE();

	virtual void Reset();

	virtual int GetPoliActionSize() const;
	virtual int GetNetOutputSize() const;
	virtual int GetNumActionFrags() const;
	virtual int GetActionFragSize() const;

	virtual bool IsExpCritic() const;
	virtual bool IsExpActor() const;

	virtual void SetExpLayer(const std::string& layer_name);
	virtual void RecordPoliAction(Eigen::VectorXd& out_action) const;
	virtual void BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

protected:
	int mNumActionFrags;
	bool mExpCritic;
	bool mExpActor;
	Eigen::VectorXd mBoltzmannBuffer;
	std::string mExpLayer;
	double mExpNoise;

	virtual void LoadNetIntern(const std::string& net_file);
	virtual void UpdateFragParams();

	virtual void BuildActionFragOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

	virtual void UpdateAction();
	virtual void DecideAction(tAction& out_action);
	virtual void ExploitPolicy(tAction& out_action);
	virtual void ExploreAction(tAction& out_action);
	virtual void DecideActionBoltzmann(tAction& out_action);

	virtual void GetRandActorAction(tAction& out_action);
	virtual void BuildActorAction(const Eigen::VectorXd& params, int a_id, tAction& out_action) const;
	virtual int BoltzmannSelectActor(const Eigen::VectorXd& params, Eigen::VectorXd& val_buffer) const;

	virtual void ApplyExpNoise(tAction& out_action);
	virtual void ApplyExpNoiseState(tAction& out_action);
	virtual bool ValidExpLayer() const;
	virtual void ApplyExpNoiseAction(tAction& out_action);

	virtual void ProcessCommand(tAction& out_action);

	virtual int GetMaxFragIdx(const Eigen::VectorXd& params) const;
	virtual double GetMaxFragVal(const Eigen::VectorXd& params) const;
	virtual void GetFrag(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action) const;
	virtual void SetFrag(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params) const;
	virtual double GetVal(const Eigen::VectorXd& params, int a_idx) const;
	virtual void SetVal(double val, int a_idx, Eigen::VectorXd& out_params) const;

	virtual void FetchExpNoiseScale(Eigen::VectorXd& out_noise) const;

	virtual void BuildActorBias(int a_id, Eigen::VectorXd& out_bias) const;

#if defined (ENABLE_DEBUG_PRINT)
	virtual void PrintNetOutput(const Eigen::VectorXd& y, int a_id) const;
#endif

#if defined(ENABLE_DEBUG_VISUALIZATION)
public:
	virtual void GetVisActionFeatures(Eigen::VectorXd& out_features) const;
	virtual void GetVisActionValues(Eigen::VectorXd& out_vals) const;

protected:
#endif
};
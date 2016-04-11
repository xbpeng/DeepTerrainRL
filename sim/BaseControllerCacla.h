#pragma once

#include "sim/DogControllerQ.h"

class cBaseControllerCacla : public virtual cTerrainRLCharController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cBaseControllerCacla();
	virtual ~cBaseControllerCacla();
	
	virtual int GetPoliActionSize() const;
	virtual void RecordPoliAction(Eigen::VectorXd& out_action) const;
	
	virtual bool ValidCritic() const;
	virtual bool LoadCriticNet(const std::string& net_file);
	virtual void LoadCriticModel(const std::string& model_file);

	virtual void CopyNet(const cNeuralNet& net);
	virtual void CopyActorNet(const cNeuralNet& net);
	virtual void CopyCriticNet(const cNeuralNet& net);

	virtual void SetExpNoise(double noise);

	virtual const cNeuralNet& GetActor() const;
	virtual cNeuralNet& GetActor();
	virtual const cNeuralNet& GetCritic() const;
	virtual cNeuralNet& GetCritic();

	virtual int GetActorInputSize() const;
	virtual int GetActorOutputSize() const;
	virtual int GetCriticInputSize() const;
	virtual int GetCriticOutputSize() const;

	virtual void BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildActorOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;
	virtual void BuildCriticOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

protected:
	cNeuralNet mCriticNet;
	double mExpNoise;

	virtual bool ShouldExplore() const;
	virtual void DecideAction(tAction& out_action);
	virtual void ExploitPolicy(tAction& out_action);
	virtual void ExploreAction(tAction& out_action);

	virtual void RecordVal();
	virtual void BuildCriticInput(Eigen::VectorXd& out_x) const;

	virtual void FetchExpNoiseScale(Eigen::VectorXd& out_noise) const;
	virtual void ApplyExpNoise(tAction& out_action);
	virtual void ApplyExpNoiseAction(tAction& out_action);
};
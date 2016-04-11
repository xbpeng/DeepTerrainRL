#pragma once

#include "NeuralNetLearner.h"

struct cACLearner : public cNeuralNetLearner
{
public:
	cACLearner(const std::shared_ptr<cNeuralNetTrainer>& trainer);
	virtual ~cACLearner();

	virtual void Init();

	virtual void SetNet(cNeuralNet* net);
	virtual const cNeuralNet* GetNet() const;
	virtual void SetActorNet(cNeuralNet* net);
	virtual const cNeuralNet* GetActorNet() const;
	virtual void SetCriticNet(cNeuralNet* net);
	virtual const cNeuralNet* GetCriticNet() const;

	virtual void LoadNet(const std::string& net_file);
	virtual void LoadSolver(const std::string& solver_file);
	virtual void LoadActorNet(const std::string& net_file);
	virtual void LoadActorSolver(const std::string& solver_file);
	virtual void LoadCriticNet(const std::string& net_file);
	virtual void LoadCriticSolver(const std::string& solver_file);

	virtual void OutputModel(const std::string& filename) const;
	virtual void OutputCritic(const std::string& filename) const;
	virtual void OutputActor(const std::string& filename) const;

	virtual void SyncNet();
	virtual bool HasCriticNet() const;

protected:
	cNeuralNet* mCriticNet;
};
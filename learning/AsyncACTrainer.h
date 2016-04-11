#pragma once

#include "learning/AsyncTrainer.h"

class cAsyncACTrainer : public cAsyncTrainer
{
public:

	virtual ~cAsyncACTrainer();

	virtual void SetActorFiles(const std::string& actor_solver_file,
								const std::string& actor_net_file);

	virtual int GetIter() const;
	virtual int GetCriticIter() const;
	virtual int GetActorIter() const;

	virtual void LoadModel(const std::string& model_file);
	virtual void LoadCriticModel(const std::string& model_file);
	virtual void LoadActorModel(const std::string& model_file);

	virtual void LoadScale(const std::string& scale_file);
	virtual void LoadCriticScale(const std::string& scale_file);
	virtual void LoadActorScale(const std::string& scale_file);

	virtual bool HasInitModel() const;
	virtual bool HasCriticInitModel() const;
	virtual bool HasActorInitModel() const;
	
	virtual int GetInputSize() const;
	virtual int GetOutputSize() const;
	virtual int GetCriticInputSize() const;
	virtual int GetCriticOutputSize() const;
	virtual int GetActorInputSize() const;
	virtual int GetActorOutputSize() const;

	virtual void SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetCriticInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetCriticOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetActorOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);

	virtual void GetCriticIDs(int& out_start, int& out_end) const;
	virtual void GetActorIDs(int& out_start, int& out_end) const;

protected:
	std::string mActorNetFile;
	std::string mActorSolverFile;

	cAsyncACTrainer();

	virtual int GetNetPoolSize() const;
	virtual void SetupNet(int id);
	virtual void SetupCriticNet(int id);
	virtual void SetupActorNet(int id);
	virtual void SetupTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer);
	
	virtual int IsCritic(int id) const;
	virtual int IsActor(int id) const;
	virtual int GetNumCritics() const;
	virtual int GetNumActors() const;
};
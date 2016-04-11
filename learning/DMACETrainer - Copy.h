#pragma once

#include "MACETrainer.h"
#include "DMACELearner.h"

class cDMACETrainer : public cMACETrainer
{
public:
	cDMACETrainer();
	virtual ~cDMACETrainer();

	virtual void SetActorFiles(const std::string& actor_solver_file,
								const std::string& actor_net_file);

	virtual void Init(const tParams& params);

	virtual void SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetCriticInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetCriticOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetActorOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	
	virtual void LoadCriticModel(const std::string& model_file);
	virtual void LoadCriticScale(const std::string& scale_file);
	virtual void LoadActorModel(const std::string& model_file);
	virtual void LoadActorScale(const std::string& scale_file);

	virtual int GetCriticInputSize() const;
	virtual int GetCriticOutputSize() const;
	virtual int GetActorInputSize() const;
	virtual int GetActorOutputSize() const;

	virtual void OutputModel(const std::string& filename) const;
	virtual void OutputCritic(const std::string& filename) const;
	virtual void OutputActor(const std::string& filename) const;

	virtual const std::string& GetNetFile() const;
	virtual const std::string& GetSolverFile() const;
	virtual const std::string& GetActorNetFile() const;
	virtual const std::string& GetActorSolverFile() const;
	virtual const std::string& GetCriticNetFile() const;
	virtual const std::string& GetCriticSolverFile() const;

	virtual const std::unique_ptr<cNeuralNet>& GetNet() const;
	virtual const std::unique_ptr<cNeuralNet>& GetCritic() const;
	virtual const std::unique_ptr<cNeuralNet>& GetActor() const;

	virtual bool HasInitModel() const;
	virtual bool HasActorInitModel() const;
	virtual bool HasCriticInitModel() const;

	virtual void SetTemp(double temp);

	virtual void RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner);

protected:
	double mTemp;

	std::string mActorSolverFile;
	std::string mActorNetFile;
	std::string mCriticSolverFile;
	std::string mCriticNetFile;

	cNeuralNet::tProblem mActorProb;
	std::unique_ptr<cNeuralNet> mActorNet;

	virtual void BuildActor(const std::string& solver_file, const std::string& net_file);
	virtual void InitActorProblem(cNeuralNet::tProblem& out_prob) const;
	virtual void BuildActorProblemY(int num_data, const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);

	virtual void OutputIntermediateModel(const std::string& filename) const;
};
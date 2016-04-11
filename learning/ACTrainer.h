#pragma once

#include "learning/NeuralNetTrainer.h"

class cACTrainer : public cNeuralNetTrainer
{
public:
	static std::string GetCriticFilename(const std::string& actor_filename);

	virtual ~cACTrainer();

	virtual void SetActorFiles(const std::string& actor_solver_file,
								const std::string& actor_net_file);
	virtual void Init(const tParams& params);
	virtual void Reset();
	virtual int AddTuple(const tExpTuple& tuple);

	virtual int GetIter() const;
	virtual int GetCriticIter() const;
	virtual int GetActorIter() const;

	virtual void SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetCriticInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetCriticOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetActorOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);

	virtual void LoadCriticModel(const std::string& model_file);
	virtual void LoadCriticScale(const std::string& scale_file);
	virtual void LoadActorModel(const std::string& model_file);
	virtual void LoadActorScale(const std::string& scale_file);

	virtual const std::string& GetNetFile() const;
	virtual const std::string& GetSolverFile() const;
	virtual const std::string& GetActorNetFile() const;
	virtual const std::string& GetActorSolverFile() const;
	virtual const std::string& GetCriticNetFile() const;
	virtual const std::string& GetCriticSolverFile() const;

	virtual int GetCriticInputSize() const;
	virtual int GetCriticOutputSize() const;
	virtual int GetActorInputSize() const;
	virtual int GetActorOutputSize() const;

	virtual int GetStateSize() const;
	virtual int GetActionSize() const;

	virtual void OutputModel(const std::string& filename) const;
	virtual void OutputCritic(const std::string& filename) const;
	virtual void OutputActor(const std::string& filename) const;

	virtual const std::unique_ptr<cNeuralNet>& GetNet() const;
	virtual const std::unique_ptr<cNeuralNet>& GetCritic() const;
	virtual const std::unique_ptr<cNeuralNet>& GetActor() const;

	virtual bool HasInitModel() const;
	virtual bool HasActorInitModel() const;
	virtual bool HasCriticInitModel() const;

	virtual void EvalNet(const tExpTuple& tuple, Eigen::VectorXd& out_y);
	virtual void EvalActor(const tExpTuple& tuple, Eigen::VectorXd& out_y);
	virtual void EvalCritic(const tExpTuple& tuple, Eigen::VectorXd& out_y);

	virtual void RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner);

protected:
	std::string mActorSolverFile;
	std::string mActorNetFile;
	std::string mCriticSolverFile;
	std::string mCriticNetFile;

	int mActorIter;
	cNeuralNet::tProblem mActorProb;
	std::unique_ptr<cNeuralNet> mActorNet;
	std::vector<int> mActorBatchBuffer;

	cACTrainer();

	virtual void InitActorProblem(cNeuralNet::tProblem& out_prob) const;
	virtual void InitStage();
	virtual void InitAvgReward();

	virtual void BuildActor(const std::string& solver_file, const std::string& net_file);

	virtual void BuildTupleActorX(const tExpTuple& tuple, Eigen::VectorXd& out_x);
	virtual void BuildTupleActorY(const tExpTuple& tuple, Eigen::VectorXd& out_y);
	virtual void BuildActorProblemX(const std::vector<int>& tuple_ids, cNeuralNet::tProblem& out_prob);
	virtual void BuildActorProblemY(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	virtual void BuildActorTupleXNext(const tExpTuple& tuple, Eigen::VectorXd& out_x);
	
	virtual void FetchActorMinibatch(int batch_size, std::vector<int>& out_batch);

	virtual bool Step();
	virtual void BuildTupleY(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y) = 0;
	virtual void BuildCriticXNext(const tExpTuple& tuple, Eigen::VectorXd& out_x);
	virtual void ApplySteps(int num_steps);
	
	virtual void UpdateMisc(const std::vector<int>& tuple_ids);
	virtual void UpdateAvgReward(const std::vector<int>& tuple_ids);

	virtual void IncActorIter();
	virtual void UpdateActorNet(const cNeuralNet::tProblem& prob);

	virtual void UpdateCurrActiveNetID();
	virtual void UpdateOffsetScale();
	virtual void UpdateCriticOffsetScale();
	virtual void UpdateActorOffsetScale();

	virtual int CalcBufferSize() const;

	virtual int GetRewardIdx() const;
	virtual int GetStateBegIdx() const;
	virtual int GetStateEndIdx() const;
	virtual int GetActionIdx() const;

	virtual void SetTuple(int t, const tExpTuple& tuple);
	virtual tExpTuple GetTuple(int t) const;

	virtual int GetActorBatchSize() const;

	virtual void UpdateActorBatchBuffer();
	virtual void UpdateActor();
	virtual void StepActor();
	virtual void BuildActorProblem(cNeuralNet::tProblem& out_prob);

	virtual void UpdateActorBatchBufferPostStep(int batch_size);

	virtual int GetServerActorID() const;
	virtual void SyncNets();
	virtual void SyncActorNet();
	virtual void ResetSolvers();
	virtual void ResetActorSolver();

	virtual void OutputIntermediateModel(const std::string& filename) const;
	virtual void UpdateParamServerCriticInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void UpdateParamServerActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
};
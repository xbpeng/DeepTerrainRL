#pragma once

#include "learning/NeuralNetTrainer.h"
#include "util/CircularBuffer.h"

#define ENABLE_ACTOR_MULTI_SAMPLE_UPDATE

class cMACETrainer : public cNeuralNetTrainer
{
public:
	enum eFlag
	{
		eFlagFail,
		eFlagExpCritic,
		eFlagExpActor,
		eFlagMax
	};

	static int GetMaxFragIdx(const Eigen::VectorXd& params, int num_frags);
	static double GetMaxFragVal(const Eigen::VectorXd& params, int num_frags);
	static void GetFrag(const Eigen::VectorXd& params, int num_frags, int frag_size, int a_idx, Eigen::VectorXd& out_action);
	static void SetFrag(const Eigen::VectorXd& frag, int a_idx, int num_frags, int frag_size, Eigen::VectorXd& out_params);
	static double GetVal(const Eigen::VectorXd& params, int a_idx);
	static void SetVal(double val, int a_idx, Eigen::VectorXd& out_params);
	static int CalcNumFrags(int param_size, int frag_size);

	// for handling the action recorded in tuples
	static int GetActionFragIdx(const Eigen::VectorXd& action_params);
	static void SetActionFragIdx(int a_idx, Eigen::VectorXd& out_action_params);
	static void GetActionFrag(const Eigen::VectorXd& action_params, Eigen::VectorXd& out_frag_params);
	static void SetActionFrag(const Eigen::VectorXd& frag_params, Eigen::VectorXd& out_action_params);

	cMACETrainer();
	virtual ~cMACETrainer();
	
	virtual void Init(const tParams& params);
	virtual void Reset();
	virtual int AddTuple(const tExpTuple& tuple);

	virtual void SetNumActionFrags(int num);
	virtual void SetActionFragSize(int size);
	virtual int GetNumActionFrags() const;
	virtual int GetActionFragSize() const;

	virtual const std::unique_ptr<cNeuralNet>& GetActor() const;

protected:
	int mNumActionFrags;
	int mActionFragSize;
	cNeuralNet::tProblem mActorProb;

	int mActorIter;
	std::vector<int> mActorBatchBuffer;
	std::vector<int> mCriticBuffer;
	std::vector<int> mActorBuffer;
	
	Eigen::MatrixXd mBatchXBuffer;
	Eigen::MatrixXd mBatchYBuffer;
	Eigen::VectorXd mBatchValBuffer0;
	Eigen::VectorXd mBatchValBuffer1;

	virtual void InitBatchBuffers();
	virtual void InitActorProblem(cNeuralNet::tProblem& out_prob) const;

	virtual void FetchMinibatch(int size, std::vector<int>& out_batch);
	virtual void FetchActorMinibatch(int size, std::vector<int>& out_batch);
	virtual void BuildNetPool(const std::string& net_file, const std::string& solver_file, int pool_size);
	virtual void BuildProblemY(int net_id, const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	virtual void BuildTupleActorY(const tExpTuple& tuple, Eigen::VectorXd& out_y);
	virtual void BuildActorProblemX(int num_data, const std::vector<int>& tuple_ids, cNeuralNet::tProblem& out_prob);
	virtual void BuildActorProblemY(int num_data, const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	virtual int GetActorBatchSize() const;

	virtual bool Step();
	virtual void BuildTupleY(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y);
	virtual void ApplySteps(int num_steps);

	virtual int GetPoolSize() const;
	virtual int GetTargetNetID(int net_id) const;
	virtual int CalcBufferSize() const;
	
	virtual int GetRewardIdx() const;
	virtual int GetStateBegIdx() const;
	virtual int GetStateEndIdx() const;
	virtual int GetActionIdx() const;
	virtual int GetActionSize() const;

	virtual double CalcCurrCumulativeReward(int net_id, const tExpTuple& tuple);
	virtual double CalcNewCumulativeReward(int net_id, const tExpTuple& tuple);
	virtual void CalcCurrCumulativeRewardBatch(int net_id, const std::vector<int>& tuple_ids, Eigen::VectorXd& out_vals);
	virtual void CalcNewCumulativeRewardBatch(int net_id, const std::vector<int>& tuple_ids, Eigen::VectorXd& out_vals);
	
	virtual void SetTuple(int t, const tExpTuple& tuple);
	virtual tExpTuple GetTuple(int t) const;

	virtual void UpdateActorBatchBuffer();
	virtual void UpdateActor();
	virtual void StepActor();
	virtual void BuildActorProblem(cNeuralNet::tProblem& out_prob);
	virtual void UpdateActorNet(const cNeuralNet::tProblem& prob);
	virtual void IncActorIter();

	virtual void UpdateBuffers(int t);
	virtual bool IsExpCritic(int t) const;
	virtual bool IsExpActor(int t) const;

	virtual bool EnableTargetNet() const;
	virtual void UpdateTargetNet();

	// for handling the (possibly separate) actor and critic outputs from network
	virtual int GetMaxFragIdxAux(const Eigen::VectorXd& params);
	virtual double GetMaxFragValAux(const Eigen::VectorXd& params);
	virtual void GetFragAux(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action);
	virtual void SetFragAux(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params);
	virtual double GetValAux(const Eigen::VectorXd& params, int a_idx);
	virtual void SetValAux(double val, int a_idx, Eigen::VectorXd& out_params);
};
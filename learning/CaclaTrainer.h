#pragma once

#include "learning/ACTrainer.h"

class cCaclaTrainer : public cACTrainer
{
public:
	enum eFlag
	{
		eFlagFail,
		eFlagOffPolicy,
		eFlagMax
	};

	enum eMode
	{
		eModeCacla,
		eModeTD,
		eModePTD,
		eModeMax
	};

	cCaclaTrainer();
	virtual ~cCaclaTrainer();

	virtual void SetMode(eMode mode);
	virtual void Init(const tParams& params);
	virtual void Reset();
	virtual int AddTuple(const tExpTuple& tuple);

	virtual void SetTDScale(double scale);
	virtual void SetActionBounds(const Eigen::VectorXd& action_min, const Eigen::VectorXd& action_max);
	
protected:
	
	eMode mMode;
	std::vector<int> mOffPolicyBuffer;

	Eigen::MatrixXd mBatchXBuffer;
	Eigen::MatrixXd mBatchYBuffer;
	Eigen::VectorXd mBatchValBuffer0;
	Eigen::VectorXd mBatchValBuffer1;

	std::vector<double> mActorBatchTDBuffer;
	double mTDScale;

	Eigen::VectorXd mActionMin;
	Eigen::VectorXd mActionMax;

	virtual void InitBatchBuffers();
	virtual void InitActorProblem(cNeuralNet::tProblem& out_prob) const;
	virtual void InitActionBounds();

	virtual void BuildNetPool(const std::string& net_file, const std::string& solver_file, int pool_size);
	virtual void FetchActorMinibatch(int batch_size, std::vector<int>& out_batch);

	virtual bool Step();
	virtual void BuildProblemY(int net_id, const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	virtual void BuildTupleY(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y);
	
	virtual int GetTargetNetID(int net_id) const;
	virtual const std::unique_ptr<cNeuralNet>& GetCriticTarget() const;

	virtual double CalcCurrCumulativeReward(const tExpTuple& tuple, const std::unique_ptr<cNeuralNet>& net);
	virtual double CalcNewCumulativeReward(const tExpTuple& tuple, const std::unique_ptr<cNeuralNet>& net);
	virtual void CalcCurrCumulativeRewardBatch(int net_id, const std::vector<int>& tuple_ids, Eigen::VectorXd& out_vals);
	virtual void CalcNewCumulativeRewardBatch(int net_id, const std::vector<int>& tuple_ids, Eigen::VectorXd& out_vals);
	
	virtual void BuildActorProblemY(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	virtual void BuildActorProblemYCacla(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	virtual void BuildActorProblemYTD(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	
	virtual int GetPoolSize() const;
	virtual void UpdateActorBatchBuffer();
	virtual void UpdateActorBatchBufferPostStep(int batch_size);

	virtual bool EnableTargetNet() const;
	virtual bool CheckUpdateTarget() const;
	virtual void UpdateTargetNet();

	virtual void UpdateBuffers(int t);
	virtual bool IsOffPolicy(int t) const;

	virtual void ProcessPoliGrad(const Eigen::VectorXd& action, Eigen::VectorXd& out_diff) const;
};
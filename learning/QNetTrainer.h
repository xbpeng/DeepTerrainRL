#pragma once

#include "learning/NeuralNetTrainer.h"

class cQNetTrainer : public cNeuralNetTrainer
{
public:
	enum eFlag
	{
		eFlagFail,
		eFlagMax
	};

	cQNetTrainer();
	virtual ~cQNetTrainer();

	virtual void Init(const tParams& params);
	virtual int AddTuple(const tExpTuple& tuple);

protected:
	Eigen::MatrixXd mBatchXBuffer;
	Eigen::MatrixXd mBatchYBuffer0;
	Eigen::MatrixXd mBatchYBuffer1;

	virtual void InitBatchBuffers();

	virtual bool Step();
	virtual void BuildProblemY(int net_id, const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	virtual void BuildTupleY(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y);

	virtual void UpdateCurrActiveNetID();
	virtual int GetNextActiveID() const;
	virtual int GetRandRefID(int id) const;

	virtual int CalcBufferSize() const;

	virtual int GetRewardIdx() const;
	virtual int GetStateBegIdx() const;
	virtual int GetStateEndIdx() const;
	virtual int GetActionIdx() const;

	virtual void SetTuple(int t, const tExpTuple& tuple);
	virtual tExpTuple GetTuple(int t) const;
};
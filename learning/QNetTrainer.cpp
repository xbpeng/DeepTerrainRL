#include "QNetTrainer.h"
#include "util/FileUtil.h"

//#define FREEZE_TARGET_NET

cQNetTrainer::cQNetTrainer()
{
}

cQNetTrainer::~cQNetTrainer()
{
}

void cQNetTrainer::Init(const tParams& params)
{
	cNeuralNetTrainer::Init(params);
	InitBatchBuffers();
}

int cQNetTrainer::AddTuple(const tExpTuple& tuple)
{
	int state_size = GetStateSize();
	assert(tuple.mStateEnd.size() == state_size);
	return cNeuralNetTrainer::AddTuple(tuple);
}

void cQNetTrainer::BuildProblemY(int net_id, const std::vector<int>& tuple_ids, 
								const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	int num_data = static_cast<int>(tuple_ids.size());
	int intput_size = GetInputSize();
	int output_size = GetOutputSize();
	assert(num_data == GetBatchSize());
	assert(out_prob.mY.rows() == num_data);

	int ref_id = GetRandRefID(net_id);
	const auto& ref_net = mNetPool[ref_id];
	const auto& curr_net = mNetPool[net_id];

	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);
		mBatchXBuffer.row(i) = tuple.mStateEnd;
	}

	ref_net->EvalBatch(mBatchXBuffer, mBatchYBuffer0);
	curr_net->EvalBatch(mBatchXBuffer, mBatchYBuffer1);
	curr_net->EvalBatch(X, out_prob.mY);

	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);
		
		double new_q = 0;
		double r = tuple.mReward;

		double discount = GetDiscount();
		double norm = CalcDiscountNorm(discount);
		r *= norm;

		int action_idx = 0;
		tuple.mAction.maxCoeff(&action_idx);

		bool fail = tuple.GetFlag(eFlagFail);
		if (fail)
		{
			new_q = r;
		}
		else
		{
			auto y_next_ref = mBatchYBuffer0.row(i);
			auto y_next = mBatchYBuffer1.row(i);

			int next_action_idx = 0;
			y_next.maxCoeff(&next_action_idx);

			double q_end = y_next_ref[next_action_idx];
			new_q = r + discount * q_end;
		}
		out_prob.mY(i, action_idx) = new_q;
	}
}

void cQNetTrainer::BuildTupleY(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	double new_q = 0;
	double r = tuple.mReward;

	double discount = GetDiscount();
	double norm = CalcDiscountNorm(discount);
	r *= norm;
	
	int action_idx = 0;
	tuple.mAction.maxCoeff(&action_idx);

	const auto& curr_net = mNetPool[net_id];
	curr_net->Eval(tuple.mStateBeg, out_y);

	bool fail = tuple.GetFlag(eFlagFail);
	if (fail)
	{
		new_q = r;
	}
	else
	{
		Eigen::VectorXd y_next;
		curr_net->Eval(tuple.mStateEnd, y_next);

		int next_action_idx = 0;
		y_next.maxCoeff(&next_action_idx);

		int ref_id = GetRandRefID(net_id);
		const auto& ref_net = mNetPool[ref_id];
		ref_net->Eval(tuple.mStateEnd, y_next);
		double q_end = y_next[next_action_idx];

		new_q = r + discount * q_end;
	}

#if defined (ENABLE_DEBUG_PRINT)
	double old_q = out_y[action_idx];
	//printf("Update action %i: old: %.5f, new: %.5f\n", action_idx, new_q, old_q);
#endif

	out_y[action_idx] = new_q;
}

void cQNetTrainer::InitBatchBuffers()
{
	int batch_size = GetBatchSize();
	if (batch_size > 0)
	{
		int input_size = GetInputSize();
		int output_size = GetOutputSize();
		mBatchXBuffer.resize(batch_size, input_size);
		mBatchYBuffer0.resize(batch_size, output_size);
		mBatchYBuffer1.resize(batch_size, output_size);
	}
}

bool cQNetTrainer::Step()
{
	int i = 0;
	int max_idx = GetPoolSize();

#if defined FREEZE_TARGET_NET
	i = mCurrActiveNet;
	max_idx = i + 1;
#endif // FREEZE_TARGET_NET

	for (i; i < max_idx; ++i)
	{
		printf("Update Net %i:\n", i);
		bool succ = BuildProblem(i, mProb);
		if (succ)
		{
			UpdateNet(i, mProb);
		}
	}

	return true;
}

void cQNetTrainer::UpdateCurrActiveNetID()
{
	mCurrActiveNet = GetNextActiveID();
}

int cQNetTrainer::GetNextActiveID() const
{
	int next_idx = 0;
	next_idx = (mCurrActiveNet + 1) % GetPoolSize();
#if defined FREEZE_TARGET_NET
	int iters = GetIter();
	if ((iters == 0) || (iters % mParams.mFreezeTargetIters != 0))
	{
		next_idx = mCurrActiveNet;
	}
#endif // FREEZE_TARGET_NET
	return next_idx;
}

int cQNetTrainer::GetRandRefID(int id) const
{
	int rand_id = 0;
	int pool_size = GetPoolSize();
	if (pool_size > 1)
	{
		rand_id = cMathUtil::RandIntExclude(0, pool_size, id);
	}
	return rand_id;
}

int cQNetTrainer::CalcBufferSize() const
{
	return 1 + GetStateSize() * 2 + GetActionSize();
}

int cQNetTrainer::GetRewardIdx() const
{
	return 0;
}

int cQNetTrainer::GetStateBegIdx() const
{
	return 1;
}

int cQNetTrainer::GetStateEndIdx() const
{
	return 1 + GetStateSize() + GetActionSize();
}

int cQNetTrainer::GetActionIdx() const
{
	return 1 + GetStateSize();
}

void cQNetTrainer::SetTuple(int t, const tExpTuple& tuple)
{
	auto curr_row = mPlaybackMem.row(t);

	curr_row(GetRewardIdx()) = static_cast<float>(tuple.mReward);

	int state_size = GetStateSize();
	int state_beg_idx = GetStateBegIdx();
	int state_end_idx = GetStateEndIdx();
	int action_idx = GetActionIdx();
	int action_size = GetActionSize();

	for (int j = 0; j < state_size; ++j)
	{
		curr_row(state_beg_idx + j) = static_cast<float>(tuple.mStateBeg(j));
		curr_row(state_end_idx + j) = static_cast<float>(tuple.mStateEnd(j));
	}

	for (int j = 0; j < action_size; ++j)
	{
		curr_row(action_idx + j) = static_cast<float>(tuple.mAction(j));
	}

	mFlagBuffer[t] = tuple.mFlags;
}

tExpTuple cQNetTrainer::GetTuple(int t) const
{
	tExpTuple tuple;
	auto curr_row = mPlaybackMem.row(t);

	tuple.mID = t;
	tuple.mReward = curr_row[GetRewardIdx()];

	int state_size = GetStateSize();
	int state_beg_idx = GetStateBegIdx();
	int state_end_idx = GetStateEndIdx();
	int action_idx = GetActionIdx();
	int action_size = GetActionSize();

	tuple.mStateBeg.resize(state_size);
	tuple.mStateEnd.resize(state_size);
	tuple.mAction.resize(action_size);

	for (int j = 0; j < state_size; ++j)
	{
		tuple.mStateBeg(j) = curr_row(state_beg_idx + j);
		tuple.mStateEnd(j) = curr_row(state_end_idx + j);
	}

	for (int j = 0; j < action_size; ++j)
	{
		tuple.mAction(j) = curr_row(action_idx + j);
	}

	tuple.mFlags = mFlagBuffer[t];

	return tuple;
}
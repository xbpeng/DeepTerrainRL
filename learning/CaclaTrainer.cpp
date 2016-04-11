#include "CaclaTrainer.h"
#include "util/FileUtil.h"
#include "QNetTrainer.h"
#include "ACLearner.h"

//#define DISABLE_EXP_BUFFER

cCaclaTrainer::cCaclaTrainer()
{
	mMode = eModeCacla;
	mTDScale = 1;
}

cCaclaTrainer::~cCaclaTrainer()
{
}

void cCaclaTrainer::SetMode(eMode mode)
{
	mMode = mode;
}

void cCaclaTrainer::Init(const tParams& params)
{
	cACTrainer::Init(params);

	mOffPolicyBuffer.clear();
	InitBatchBuffers();
	InitActionBounds();
}

void cCaclaTrainer::Reset()
{
	cACTrainer::Reset();
	mOffPolicyBuffer.clear();
	mActorBatchTDBuffer.clear();
}

int cCaclaTrainer::AddTuple(const tExpTuple& tuple)
{
	int t = cACTrainer::AddTuple(tuple);
	UpdateBuffers(t);
	return t;
}

void cCaclaTrainer::SetTDScale(double scale)
{
	mTDScale = scale;
}

void cCaclaTrainer::SetActionBounds(const Eigen::VectorXd& action_min, const Eigen::VectorXd& action_max)
{
	mActionMin = action_min;
	mActionMax = action_max;
}

void cCaclaTrainer::InitBatchBuffers()
{
	int batch_size = GetBatchSize();
	if (batch_size > 0)
	{
		int input_size = GetInputSize();
		int output_size = GetOutputSize();
		mBatchXBuffer.resize(batch_size, input_size);
		mBatchYBuffer.resize(batch_size, output_size);
		mBatchValBuffer0.resize(batch_size);
		mBatchValBuffer1.resize(batch_size);
	}
	mActorBatchTDBuffer.clear();
}

void cCaclaTrainer::InitActorProblem(cNeuralNet::tProblem& out_prob) const
{
	const auto& curr_net = GetActor();
	const int x_size = curr_net->GetInputSize();
	const int y_size = curr_net->GetOutputSize();
	int num_data = GetBatchSize();

	out_prob.mX.resize(num_data, x_size);
	out_prob.mY.resize(num_data, y_size);
	out_prob.mPassesPerStep = 1;
}

void cCaclaTrainer::InitActionBounds()
{
	int action_size = GetActionSize();
	mActionMin = -std::numeric_limits<double>::infinity() * Eigen::VectorXd::Ones(action_size);
	mActionMax = std::numeric_limits<double>::infinity() * Eigen::VectorXd::Ones(action_size);
}

void cCaclaTrainer::BuildNetPool(const std::string& net_file, const std::string& solver_file, int pool_size)
{
	cACTrainer::BuildNetPool(net_file, solver_file, pool_size);
	if (EnableTargetNet())
	{
		for (int i = 0; i < GetNetPoolSize(); ++i)
		{
			auto& net = mNetPool[i];
			auto& target_net = GetTargetNet(i);
			target_net->CopyModel(*net.get());
		}
	}
}

void cCaclaTrainer::FetchActorMinibatch(int batch_size, std::vector<int>& out_batch)
{
	int num_exp_tuples = static_cast<int>(mOffPolicyBuffer.size());
	int num_samples = std::min(batch_size, num_exp_tuples);
	out_batch.clear();
	out_batch.reserve(num_samples);

	for (int i = 0; i < num_samples; ++i)
	{
		int rand_idx = cMathUtil::RandInt(0, num_exp_tuples);
		int t = mOffPolicyBuffer[rand_idx];

#if defined(DISABLE_EXP_BUFFER)
		t = cMathUtil::RandInt(0, mNumTuples);
#endif
		bool contains = (std::find(mActorBatchBuffer.begin(), mActorBatchBuffer.end(), t) != mActorBatchBuffer.end())
						|| (std::find(out_batch.begin(), out_batch.end(), t) != out_batch.end());
		if (!contains)
		{
			out_batch.push_back(t);
		}
	}
}

void cCaclaTrainer::BuildTupleY(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	const auto& target_net = GetTargetNet(net_id);
	double new_v = CalcNewCumulativeReward(tuple, target_net);
	out_y = Eigen::VectorXd::Zero(GetOutputSize());
	out_y[0] = new_v;
}

bool cCaclaTrainer::Step()
{
	bool succ = cACTrainer::Step();

	if (CheckUpdateTarget())
	{
		UpdateTargetNet();
	}
	
	return succ;
}

void cCaclaTrainer::BuildProblemY(int net_id, const std::vector<int>& tuple_ids,
									const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	int num_data = static_cast<int>(tuple_ids.size());
	assert(num_data == GetBatchSize());
	assert(out_prob.mY.rows() == num_data);
	CalcNewCumulativeRewardBatch(net_id, tuple_ids, mBatchValBuffer0);
	out_prob.mY = mBatchValBuffer0;
}

int cCaclaTrainer::GetTargetNetID(int net_id) const
{
	int target_id = net_id;
	if (EnableTargetNet())
	{
		target_id = mParams.mPoolSize + net_id;
	}
	return target_id;
}

const std::unique_ptr<cNeuralNet>& cCaclaTrainer::GetCriticTarget() const
{
	return GetTargetNet(mCurrActiveNet);
}

double cCaclaTrainer::CalcCurrCumulativeReward(const tExpTuple& tuple, const std::unique_ptr<cNeuralNet>& net)
{
	Eigen::VectorXd x;
	BuildTupleX(tuple, x);

	Eigen::VectorXd y_next;
	net->Eval(x, y_next);
	double val = y_next[0];

	return val;
}

double cCaclaTrainer::CalcNewCumulativeReward(const tExpTuple& tuple, const std::unique_ptr<cNeuralNet>& net)
{
	double val = 0;
	double r = tuple.mReward;

	double discount = GetDiscount();
	double norm_r = NormalizeReward(r);

	bool fail = tuple.GetFlag(eFlagFail);
	if (fail)
	{
		val = norm_r;
	}
	else
	{
		Eigen::VectorXd x_next;
		BuildCriticXNext(tuple, x_next);

		Eigen::VectorXd y_next;
		net->Eval(x_next, y_next);

		double v_end = y_next[0];
		val = norm_r + discount * v_end;
	}

	return val;
}

void cCaclaTrainer::CalcCurrCumulativeRewardBatch(int net_id, const std::vector<int>& tuple_ids,
													Eigen::VectorXd& out_vals)
{
	const int num_data = static_cast<int>(tuple_ids.size());
	assert(num_data <= GetBatchSize());
	const auto& tar_net = GetTargetNet(net_id);

	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);
		Eigen::VectorXd x;
		BuildTupleX(tuple, x);
		mBatchXBuffer.row(i) = x;
	}

	tar_net->EvalBatch(mBatchXBuffer, mBatchYBuffer);
	out_vals = mBatchYBuffer;
}

void cCaclaTrainer::CalcNewCumulativeRewardBatch(int net_id, const std::vector<int>& tuple_ids,
												Eigen::VectorXd& out_vals)
{
	const int num_data = static_cast<int>(tuple_ids.size());
	assert(num_data <= GetBatchSize());
	const auto& tar_net = GetTargetNet(net_id);

	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);
		
		Eigen::VectorXd x_next;
		BuildCriticXNext(tuple, x_next);
		mBatchXBuffer.row(i) = x_next;
	}

	tar_net->EvalBatch(mBatchXBuffer, mBatchYBuffer);

	double discount = GetDiscount();

	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		double val = 0;
		double r = tuple.mReward;
		double norm_r = NormalizeReward(r);

		bool fail = tuple.GetFlag(eFlagFail);
		if (fail)
		{
			val = norm_r;
		}
		else
		{
			double v_end = mBatchYBuffer(i, 0);
			val = norm_r + discount * v_end;
		}

		out_vals(i) = val;
	}
}

void cCaclaTrainer::BuildActorProblemY(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	switch (mMode)
	{
	case eModeCacla:
		BuildActorProblemYCacla(tuple_ids, X, out_prob);
		break;
	case eModeTD:
	case eModePTD:
		BuildActorProblemYTD(tuple_ids, X, out_prob);
		break;
	default:
		assert(false); // unsupported mode
		break;
	}
}

void cCaclaTrainer::BuildActorProblemYCacla(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	cACTrainer::BuildActorProblemY(tuple_ids, X, out_prob);
}

void cCaclaTrainer::BuildActorProblemYTD(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	auto& actor_net = GetActor();
	actor_net->EvalBatch(X, out_prob.mY);

	int num_tuples = static_cast<int>(tuple_ids.size());
	int batch_size = GetActorBatchSize();
	batch_size = std::min(batch_size, num_tuples);

	for (int i = 0; i < batch_size; ++i)
	{
		int t = tuple_ids[i];
		const tExpTuple& tuple = GetTuple(t);
		double td = mActorBatchTDBuffer[i];

		printf("TD: %.5f\n", td);
		td *= mTDScale;
		
		Eigen::VectorXd curr_action = out_prob.mY.row(i);
		Eigen::VectorXd new_action;
		BuildTupleActorY(tuple, new_action);

		Eigen::VectorXd diff = new_action - curr_action;
		diff *= td;

		ProcessPoliGrad(curr_action, diff);
		new_action = curr_action + diff;
		out_prob.mY.row(i) = new_action;
	}
}

int cCaclaTrainer::GetPoolSize() const
{
	int pool_size = GetNetPoolSize();
	if (EnableTargetNet())
	{
		pool_size *= 2;
	}
	return pool_size;
}

void cCaclaTrainer::UpdateActorBatchBuffer()
{
	int batch_size = GetActorBatchSize();
	FetchActorMinibatch(batch_size, mBatchBuffer);
	int num_samples = static_cast<int>(mBatchBuffer.size());
	Eigen::VectorXd pass_buffer = Eigen::VectorXd::Ones(num_samples);
	Eigen::VectorXd td_buffer = Eigen::VectorXd::Zero(num_samples);

	int net_pool_size = GetNetPoolSize();
	for (int k = 0; k < net_pool_size; ++k)
	{
		int net_id = k;
		CalcCurrCumulativeRewardBatch(net_id, mBatchBuffer, mBatchValBuffer0);
		CalcNewCumulativeRewardBatch(net_id, mBatchBuffer, mBatchValBuffer1);

		for (int i = 0; i < num_samples; ++i)
		{
			double curr_val = mBatchValBuffer0[i];
			double new_val = mBatchValBuffer1[i];
			double td = new_val - curr_val;
			td_buffer[i] += td;

			if (mMode == eModeCacla || mMode == eModePTD)
			{
				if (td <= 0)
				{
					pass_buffer[i] = 0;
				}
			}
		}
	}
	td_buffer /= net_pool_size;

	for (int i = 0; i < num_samples; ++i)
	{
		double curr_val = pass_buffer[i];
		if (curr_val != 0)
		{
			int t = mBatchBuffer[i];
			double td = td_buffer[i];
			
			mActorBatchBuffer.push_back(t);
			mActorBatchTDBuffer.push_back(td);
		}
	}
}

void cCaclaTrainer::UpdateActorBatchBufferPostStep(int batch_size)
{
	cACTrainer::UpdateActorBatchBufferPostStep(batch_size);
	mActorBatchTDBuffer.erase(mActorBatchTDBuffer.begin(), mActorBatchTDBuffer.begin() + batch_size);
}

bool cCaclaTrainer::EnableTargetNet() const
{
	bool enable = (mParams.mFreezeTargetIters > 0);
	return enable;
}

bool cCaclaTrainer::CheckUpdateTarget() const
{
	int iters = GetIter();
	return EnableTargetNet() && (iters > 0) && (iters % mParams.mFreezeTargetIters == 0);
}

void cCaclaTrainer::UpdateTargetNet()
{
	if (EnableTargetNet())
	{
		for (int i = 0; i < GetNetPoolSize(); ++i)
		{
			auto& net = mNetPool[i];
			auto& target_net = GetTargetNet(i);
			target_net->CopyModel(*net.get());
		}
	}
}

void cCaclaTrainer::UpdateBuffers(int t)
{
	if (t != gInvalidIdx)
	{
		bool is_off_policy = IsOffPolicy(t);

		auto off_beg = mOffPolicyBuffer.begin();
		auto off_end = mOffPolicyBuffer.end();
		auto off_iter = std::find(off_beg, off_end, t);

		bool off_contains = off_iter != off_end;
		if (is_off_policy)
		{
			if (!off_contains)
			{
				mOffPolicyBuffer.push_back(t);
			}
		}
		else if (off_contains)
		{
			int idx = static_cast<int>(off_iter - off_beg);
			int last_val = mOffPolicyBuffer[mOffPolicyBuffer.size() - 1];
			mOffPolicyBuffer[idx] = last_val;
			mOffPolicyBuffer.pop_back();
		}

		auto actor_buffer_iter = std::find(mActorBatchBuffer.begin(), mActorBatchBuffer.end(), t);
		while (actor_buffer_iter != mActorBatchBuffer.end())
		{
			int idx = static_cast<int>(actor_buffer_iter - mActorBatchBuffer.begin());
			int last_id = mActorBatchBuffer[mActorBatchBuffer.size() - 1];
			double last_td = mActorBatchTDBuffer[mActorBatchTDBuffer.size() - 1];

			mActorBatchBuffer[idx] = last_id;
			mActorBatchTDBuffer[idx] = last_td;
			mActorBatchBuffer.pop_back();
			mActorBatchTDBuffer.pop_back();

			actor_buffer_iter = std::find(mActorBatchBuffer.begin(), mActorBatchBuffer.end(), t);
		}
	}
	assert(mActorBatchBuffer.size() == mActorBatchTDBuffer.size());
}

bool cCaclaTrainer::IsOffPolicy(int t) const
{
	int flag = mFlagBuffer[t];
	bool off_policy = tExpTuple::TestFlag(flag, eFlagOffPolicy);
	return off_policy;
}


void cCaclaTrainer::ProcessPoliGrad(const Eigen::VectorXd& action, Eigen::VectorXd& out_diff) const
{
	int action_size = static_cast<int>(action.size());
	assert(action.size() == action.size());
	assert(action.size() == mActionMin.size());
	assert(action.size() == mActionMax.size());

	for (int i = 0; i < action_size; ++i)
	{
		double diff_val = out_diff[i];
		double action_val = action[i];
		double tar_val = action_val + diff_val;

		double bound_min = mActionMin[i];
		double bound_max = mActionMax[i];
		
		if (diff_val > 0 && tar_val > bound_max)
		{
			diff_val = bound_max - action_val;
		}
		else if (diff_val < 0 && tar_val < bound_min)
		{
			diff_val = bound_min - action_val;
		}

		out_diff[i] = diff_val;
	}
}
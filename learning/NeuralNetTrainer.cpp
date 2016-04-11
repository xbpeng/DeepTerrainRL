#include "NeuralNetTrainer.h"
#include "util/FileUtil.h"
#include "util/Util.h"

//#define DISABLE_EXP_REPLAY

const std::string gLogFile = "output/logs/trainer_log.txt";

double cNeuralNetTrainer::CalcDiscountNorm(double discount)
{
	double norm = (1 - discount);
	return norm;
}

cNeuralNetTrainer::cNeuralNetTrainer()
{
	ResetParams();
	mParamServer = nullptr;
	mAvgReward = 0.5;
}

cNeuralNetTrainer::~cNeuralNetTrainer()
{
}

void cNeuralNetTrainer::Init(const tParams& params)
{
	mParams = params;
	int pool_size = GetPoolSize();
	BuildNetPool(params.mNetFile, params.mSolverFile, pool_size);
	InitPlaybackMem(params.mPlaybackMemSize);

	ResetParams();
	InitBatchBuffer();
	InitProblem(mProb);

	if (EnableAsyncMode())
	{
		SyncNets();
	}
}

void cNeuralNetTrainer::LoadModel(const std::string& model_file)
{
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		mNetPool[i]->LoadModel(model_file);
	}
}

void cNeuralNetTrainer::LoadScale(const std::string& scale_file)
{
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		mNetPool[i]->LoadScale(scale_file);
	}
}

void cNeuralNetTrainer::Reset()
{
	ResetParams();
	int pool_size = GetNetPoolSize();
	BuildNetPool(mParams.mNetFile, mParams.mSolverFile, pool_size);

	if (EnableAsyncMode())
	{
		SyncNets();
	}

	ResetLearners();
}

void cNeuralNetTrainer::EndTraining()
{
	mDone = true;

#if defined(OUTPUT_TRAINER_LOG)
	EndLog();
	WriteLog(gLogFile);
#endif
}

void cNeuralNetTrainer::RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner)
{
	out_learner = std::shared_ptr<cNeuralNetLearner>(new cNeuralNetLearner(shared_from_this()));
}

int cNeuralNetTrainer::RegisterLearner(cNeuralNetLearner* learner)
{
	int id = gInvalidIdx;
	auto iter = std::find(mLearners.begin(), mLearners.end(), learner);
	if (iter != mLearners.end())
	{
		assert(false); // learner already registered
	}
	else
	{
		id = static_cast<int>(mLearners.size());
		mLearners.push_back(learner);
	}
	return id;
}

void cNeuralNetTrainer::UnregisterLearner(cNeuralNetLearner* learner)
{
	auto iter = std::find(mLearners.begin(), mLearners.end(), learner);
	if (iter != mLearners.end())
	{
		size_t idx = iter - mLearners.begin();
		mLearners[idx] = mLearners[GetNumLearners() - 1];
		mLearners.pop_back();
	}
	else
	{
		assert(false); // learner not found
	}
}

bool cNeuralNetTrainer::EnableAsyncMode() const
{
	return mParamServer != nullptr;
}

void cNeuralNetTrainer::Lock()
{
#if defined(OUTPUT_TRAINER_LOG)
	TIMER_RECORD_BEG(LOCK_WAIT)
#endif
	mLock.lock();
#if defined(OUTPUT_TRAINER_LOG)
	TIMER_RECORD_END(LOCK_WAIT, mLog.mLockWaitTime, mLog.mLockWaitSamples)
#endif
}

void cNeuralNetTrainer::Unlock()
{
	mLock.unlock();
}

void cNeuralNetTrainer::SetParamServer(cParamServer* server)
{
	mParamServer = server;
}

int cNeuralNetTrainer::AddTuple(const tExpTuple& tuple)
{
	int state_size = GetStateSize();
	int action_size = GetActionSize();
	assert(tuple.mStateBeg.size() == state_size);
	assert(tuple.mAction.size() == action_size);

	int id = gInvalidIdx;
	bool valid_tuple = CheckTuple(tuple);
	if (valid_tuple)
	{
		id = mBufferHead;
		SetTuple(mBufferHead, tuple);

		int buffer_size = GetPlaybackMemSize();
		mBufferHead = (mBufferHead + 1) % buffer_size;
		mNumTuples = std::min(buffer_size, mNumTuples + 1);
		++mTotalTuples;
	}
	return id;
}

void cNeuralNetTrainer::AddTuples(const std::vector<tExpTuple>& tuples)
{
	for (size_t i = 0; i < tuples.size(); ++i)
	{
		AddTuple(tuples[i]);
	}
}

void cNeuralNetTrainer::Train()
{
	UpdateStage();

	if (mStage == eStageTrain)
	{
		ApplySteps(mParams.mNumStepsPerIter);
	}
}

const std::unique_ptr<cNeuralNet>& cNeuralNetTrainer::GetNet() const
{
	return GetCurrNet();
}

const std::unique_ptr<cNeuralNet>& cNeuralNetTrainer::GetCurrNet() const
{
	return mNetPool[mCurrActiveNet];
}

double cNeuralNetTrainer::GetDiscount() const
{
	double discount = (mParams.mRewardMode == eRewardModeAvg) ? 1 : mParams.mDiscount;
	return discount;
}

double cNeuralNetTrainer::GetAvgReward() const
{
	return mAvgReward;
}

int cNeuralNetTrainer::GetIter() const
{
	return mIter;
}

double cNeuralNetTrainer::NormalizeReward(double r) const
{
	double norm_r = r;

	switch (mParams.mRewardMode)
	{
	case eRewardModeStart:
		norm_r = r * CalcDiscountNorm(GetDiscount());
		break;
	case eRewardModeAvg:
		norm_r = r - GetAvgReward();
		break;
	default:
		assert(false); // unsupported reward mode
		break;
	}

	return norm_r;
}

void cNeuralNetTrainer::SetNumInitSamples(int num)
{
	mParams.mNumInitSamples = num;
}

void cNeuralNetTrainer::SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	const auto& curr_net = GetCurrNet();
	int offset_size = static_cast<int>(offset.size());
	int scale_size = static_cast<int>(scale.size());
	int num_inputs = curr_net->GetInputSize();

	assert(offset_size == num_inputs);
	assert(scale_size == num_inputs);
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		mNetPool[i]->SetInputOffsetScale(offset, scale);
	}
}

void cNeuralNetTrainer::SetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	const auto& curr_net = GetCurrNet();
	int offset_size = static_cast<int>(offset.size());
	int scale_size = static_cast<int>(scale.size());
	int num_outputs = curr_net->GetOutputSize();

	assert(offset_size == num_outputs);
	assert(scale_size == num_outputs);
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		mNetPool[i]->SetOutputOffsetScale(offset, scale);
	}
}

int cNeuralNetTrainer::GetNumInitSamples() const
{
	return std::min(mParams.mNumInitSamples, GetPlaybackMemSize());
}

const std::string& cNeuralNetTrainer::GetNetFile() const
{
	return mParams.mNetFile;
}

const std::string& cNeuralNetTrainer::GetSolverFile() const
{
	return mParams.mSolverFile;
}

cNeuralNetTrainer::eStage cNeuralNetTrainer::GetStage() const
{
	return mStage;
}

int cNeuralNetTrainer::GetNumTuples() const
{
	return mTotalTuples;
}

void cNeuralNetTrainer::OutputModel(const std::string& filename) const
{
	const auto& curr_net = GetCurrNet();
	curr_net->OutputModel(filename);
}

bool cNeuralNetTrainer::HasInitModel() const
{
	bool has_init_model = false;
	const auto& curr_net = GetCurrNet();
	if (curr_net != nullptr)
	{
		has_init_model = curr_net->HasValidModel();
	}
	return has_init_model;
}

void cNeuralNetTrainer::EvalNet(const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	Eigen::VectorXd x;
	BuildTupleX(tuple, x);
	const auto& net = GetNet();
	net->Eval(x, out_y);
}

int cNeuralNetTrainer::GetStateSize() const
{
	int size = 0;
	const auto& curr_net = GetCurrNet();
	if (curr_net->HasNet())
	{
		size = curr_net->GetInputSize();
	}
	return size;
}

int cNeuralNetTrainer::GetActionSize() const
{
	int size = 0;
	const std::unique_ptr<cNeuralNet>& curr_net = GetCurrNet();
	if (curr_net->HasNet())
	{
		size = curr_net->GetOutputSize();
	}
	return size;
}

int cNeuralNetTrainer::GetInputSize() const
{
	const auto& curr_net = GetCurrNet();
	return curr_net->GetInputSize();
}

int cNeuralNetTrainer::GetOutputSize() const
{
	const auto& curr_net = GetCurrNet();
	return curr_net->GetOutputSize();
}

int cNeuralNetTrainer::GetBatchSize() const
{
	const auto& curr_net = GetCurrNet();
	return curr_net->GetBatchSize();
}

void cNeuralNetTrainer::InitPlaybackMem(int size)
{
	mPlaybackMem.resize(size, CalcBufferSize());
	mFlagBuffer.resize(size);
}

void cNeuralNetTrainer::InitBatchBuffer()
{
	int batch_size = GetBatchSize();
	mBatchBuffer.resize(batch_size);
}

void cNeuralNetTrainer::InitProblem(cNeuralNet::tProblem& out_prob) const
{
	const auto& curr_net = GetCurrNet();
	const int x_size = curr_net->GetInputSize();
	const int y_size = curr_net->GetOutputSize();
	int num_data = GetBatchSize();

	out_prob.mX.resize(num_data, x_size);
	out_prob.mY.resize(num_data, y_size);
	out_prob.mPassesPerStep = 1;
}

int cNeuralNetTrainer::GetPlaybackMemSize() const
{
	return static_cast<int>(mPlaybackMem.rows());
}

void cNeuralNetTrainer::ResetParams()
{
	mTotalTuples = 0;
	mNumTuples = 0;
	mCurrActiveNet = 0;
	mBufferHead = 0;
	mIter = 0;
	mStage = eStageInit;
}

void cNeuralNetTrainer::Pretrain()
{
}

bool cNeuralNetTrainer::Step()
{
	bool succ = false;
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		printf("Update Net %i:\n", i);
		succ = BuildProblem(i, mProb);
		if (succ)
		{
			UpdateNet(i, mProb);
		}
	}
	return succ;
}

bool cNeuralNetTrainer::BuildProblem(int net_id, cNeuralNet::tProblem& out_prob)
{
	bool succ = true;
	int num_data = GetBatchSize();
	FetchMinibatch(num_data, mBatchBuffer);

	if (mBatchBuffer.size() >= num_data)
	{
		{
#if defined(OUTPUT_TRAINER_LOG)
			TIMER_RECORD_BEG(BUILD_TUPLE_X)
#endif
			BuildProblemX(net_id, mBatchBuffer, out_prob);
#if defined(OUTPUT_TRAINER_LOG)
			{
				std::lock_guard<std::mutex> lock(mLogLock);
				TIMER_RECORD_END(BUILD_TUPLE_X, mLog.mBuildTupleXTime, mLog.mBuildTupleXSamples)
			}
#endif
		}

		{
#if defined(OUTPUT_TRAINER_LOG)
			TIMER_RECORD_BEG(BUILD_TUPLE_Y)
#endif
			BuildProblemY(net_id, mBatchBuffer, out_prob.mX, out_prob);
#if defined(OUTPUT_TRAINER_LOG)
			{
				std::lock_guard<std::mutex> lock(mLogLock);
				TIMER_RECORD_END(BUILD_TUPLE_Y, mLog.mBuildTupleYTime, mLog.mBuildTupleYSamples)
			}
#endif
		}

		UpdateMisc(mBatchBuffer);
	}
	else
	{
		succ = false;
	}

	return succ;
}

void cNeuralNetTrainer::BuildProblemX(int net_id, const std::vector<int>& tuple_ids, cNeuralNet::tProblem& out_prob)
{
	int num_data = static_cast<int>(tuple_ids.size());
	assert(num_data == GetBatchSize());
	assert(out_prob.mX.rows() == num_data);
	
	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		Eigen::VectorXd x;
		BuildTupleX(tuple, x);
		out_prob.mX.row(i) = x;
	}
}

void cNeuralNetTrainer::BuildProblemY(int net_id, const std::vector<int>& tuple_ids, 
									const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	int num_data = static_cast<int>(tuple_ids.size());
	assert(num_data == GetBatchSize());
	assert(out_prob.mY.rows() == num_data);

	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		Eigen::VectorXd y;
		BuildTupleY(net_id, tuple, y);
		out_prob.mY.row(i) = y;
	}
}

void cNeuralNetTrainer::UpdateMisc(const std::vector<int>& tuple_ids)
{
}

void cNeuralNetTrainer::BuildTupleX(const tExpTuple& tuple, Eigen::VectorXd& out_x)
{
	out_x = tuple.mStateBeg;
}


void cNeuralNetTrainer::BuildTupleY(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	out_y = tuple.mAction;
}

void cNeuralNetTrainer::FetchMinibatch(int size, std::vector<int>& out_batch)
{
	out_batch.resize(size);
	for (int i = 0; i < size; ++i)
	{
#if defined(DISABLE_EXP_REPLAY)
		int t = mBufferHead - i - 1;
		if (t < 0)
		{
			t = mNumTuples + t;
		}
#else
		int t = cMathUtil::RandInt(0, mNumTuples);
#endif
		out_batch[i] = t;
	}
}

int cNeuralNetTrainer::GetTargetNetID(int net_id) const
{
	return net_id;
}

void cNeuralNetTrainer::UpdateCurrActiveNetID()
{
}

const std::unique_ptr<cNeuralNet>& cNeuralNetTrainer::GetTargetNet(int net_id) const
{
	int target_net_id = GetTargetNetID(net_id);
	return mNetPool[target_net_id];
}

bool cNeuralNetTrainer::CheckTuple(const tExpTuple& tuple) const
{
	if (!std::isfinite(tuple.mReward))
	{
		return false;
	}

	for (int i = 0; i < static_cast<int>(tuple.mStateBeg.size()); ++i)
	{
		double curr_val = tuple.mStateBeg[i];
		if (!std::isfinite(curr_val))
		{
			return false;
		}
	}

	for (int i = 0; i < static_cast<int>(tuple.mStateEnd.size()); ++i)
	{
		double curr_val = tuple.mStateEnd[i];
		if (!std::isfinite(curr_val))
		{
			return false;
		}
	}

	for (int i = 0; i < static_cast<int>(tuple.mAction.size()); ++i)
	{
		double curr_val = tuple.mAction[i];
		if (!std::isfinite(curr_val))
		{
			return false;
		}
	}

	return true;
}

void cNeuralNetTrainer::UpdateNet(int net_id, const cNeuralNet::tProblem& prob)
{
#if defined(OUTPUT_TRAINER_LOG)
	TIMER_RECORD_BEG(UPDATE_NET)
#endif
	auto& curr_net = mNetPool[net_id];
	if (EnableAsyncMode())
	{
#if defined(OUTPUT_TRAINER_LOG)
		TIMER_RECORD_BEG(ASYNC_FORWARD_BACKWARD)
#endif
		double loss = curr_net->ForwardBackward(prob);
		printf("Net %i Loss: %.8f\n", net_id, loss);

#if defined(OUTPUT_TRAINER_LOG)
		TIMER_RECORD_END(ASYNC_FORWARD_BACKWARD, mLog.mAsyncForwardBackTime, mLog.mAsyncForwardBackSamples)
#endif

#if defined(OUTPUT_TRAINER_LOG)
		TIMER_RECORD_BEG(ASYNC_UPDATE_NET)
#endif
		cParamServer::tInputInfo server_input;
		server_input.mID = net_id;
		server_input.mGradNet = curr_net.get();

		cParamServer::tOutputInfo server_output;
		server_output.mSyncNet = curr_net.get();

		mParamServer->UpdateNet(server_input, server_output);
		
#if defined(OUTPUT_TRAINER_LOG)
		TIMER_RECORD_END(ASYNC_UPDATE_NET, mLog.mAsyncUpdateNetTime, mLog.mAsyncUpdateNetSamples)
#endif

		if (net_id == 0)
		{
			mIter = server_output.mIter;
		}
	}
	else
	{
		curr_net->Train(prob);
	}
#if defined(OUTPUT_TRAINER_LOG)
	{
		std::lock_guard<std::mutex> lock(mLogLock);
		TIMER_RECORD_END(UPDATE_NET, mLog.mUpdateNetTime, mLog.mUpdateNetSamples)
	}
#endif
}

int cNeuralNetTrainer::CalcBufferSize() const
{
	return GetStateSize() + GetActionSize();
}

int cNeuralNetTrainer::GetStateBegIdx() const
{
	return 0;
}

int cNeuralNetTrainer::GetActionIdx() const
{
	return GetStateSize();
}

void cNeuralNetTrainer::SetTuple(int t, const tExpTuple& tuple)
{
	auto curr_row = mPlaybackMem.row(t);

	int state_beg_idx = GetStateBegIdx();
	int state_size = GetStateSize();
	int action_idx = GetActionIdx();
	int action_size = GetActionSize();

	for (int j = 0; j < state_size; ++j)
	{
		curr_row(state_beg_idx + j) = static_cast<float>(tuple.mStateBeg(j));
	}

	for (int j = 0; j < action_size; ++j)
	{
		curr_row(action_idx + j) = static_cast<float>(tuple.mAction(j));
	}

	mFlagBuffer[t] = tuple.mFlags;
}

tExpTuple cNeuralNetTrainer::GetTuple(int t) const
{
	tExpTuple tuple;
	auto curr_row = mPlaybackMem.row(t);

	int state_beg_idx = GetStateBegIdx();
	int state_size = GetStateSize();
	int action_idx = GetActionIdx();
	int action_size = GetActionSize();

	tuple.mID = t;
	tuple.mStateBeg.resize(state_size);
	tuple.mStateEnd.resize(0);
	tuple.mAction.resize(action_size);

	for (int j = 0; j < state_size; ++j)
	{
		tuple.mStateBeg(j) = curr_row(state_beg_idx + j);
	}

	for (int j = 0; j < action_size; ++j)
	{
		tuple.mAction(j) = curr_row(action_idx + j);
	}
	
	tuple.mFlags = mFlagBuffer[t];

	return tuple;
}

void cNeuralNetTrainer::UpdateOffsetScale()
{
	const auto& curr_net = GetCurrNet();
	int state_size = GetStateSize();
	Eigen::MatrixXd X(mNumTuples, state_size);

	Eigen::VectorXd x;
	for (int i = 0; i < mNumTuples; ++i)
	{
		tExpTuple tuple = GetTuple(i);
		BuildTupleX(tuple, x);
		X.row(i) = x;
	}

	Eigen::VectorXd offset;
	Eigen::VectorXd scale;
	curr_net->CalcOffsetScale(X, offset, scale);
	SetInputOffsetScale(offset, scale);

	if (EnableAsyncMode())
	{
		UpdateParamServerInputOffsetScale(offset, scale);
	}
}

void cNeuralNetTrainer::UpdateStage()
{
	if (mStage == eStageInit)
	{
		int num_init_samples = GetNumInitSamples();
		if (mNumTuples >= num_init_samples && mNumTuples > 0)
		{
			InitStage();
			mStage = eStageTrain;
		}
	}
}

void cNeuralNetTrainer::InitStage()
{
	int batch_size = GetBatchSize();
	int num_init_samples = GetNumInitSamples();
	if (num_init_samples > 1 && mParams.mInitInputOffsetScale)
	{
		UpdateOffsetScale();
	}
	Pretrain();

#if defined(OUTPUT_TRAINER_LOG)
	InitLog();
#endif // OUTPUT_TRAINER_LOG
}

void cNeuralNetTrainer::ApplySteps(int num_steps)
{
	if (EnableIntOutput())
	{
		OutputIntermediate();
	}
	
	bool succ_step = false;
	for (int i = 0; i < num_steps; ++i)
	{
#if defined(OUTPUT_TRAINER_LOG)
		TIMER_RECORD_BEG(TRAIN_STEP)
#endif
		succ_step = Step();
#if defined(OUTPUT_TRAINER_LOG)
		{
			std::lock_guard<std::mutex> lock(mLogLock);
			TIMER_RECORD_END(TRAIN_STEP, mLog.mStepTime, mLog.mStepSamples)
		}
#endif
	}

	if (succ_step)
	{
		IncIter();
		UpdateCurrActiveNetID();
	}
}

void cNeuralNetTrainer::IncIter()
{
	if (!EnableAsyncMode())
	{
		++mIter;
	}
}

int cNeuralNetTrainer::GetNetPoolSize() const
{
	return mParams.mPoolSize;
}

void cNeuralNetTrainer::BuildNetPool(const std::string& net_file, const std::string& solver_file,
								int pool_size)
{
	assert(pool_size > 0);
	pool_size = std::max(1, pool_size);
	mNetPool.clear();
	mNetPool.resize(pool_size);

	for (int i = 0; i < pool_size; ++i)
	{
		auto& net = mNetPool[i];
		net = std::unique_ptr<cNeuralNet>(new cNeuralNet());
		net->LoadNet(net_file);
		net->LoadSolver(solver_file);
	}
}

int cNeuralNetTrainer::GetPoolSize() const
{
	return mParams.mPoolSize;
}

bool cNeuralNetTrainer::EnableIntOutput() const
{
	return (mParams.mIntOutputFile != "") && (mParams.mIntOutputIters > 0);
}

void cNeuralNetTrainer::OutputIntermediate()
{
	int curr_iter = GetIter();
	if (curr_iter % mParams.mIntOutputIters == 0)
	{
		std::string ext = cFileUtil::GetExtension(mParams.mIntOutputFile);
		std::string filename_noext = cFileUtil::RemoveExtension(mParams.mIntOutputFile);

		char str_buffer[128];
		sprintf(str_buffer, "%010d", curr_iter);
		std::string filename = filename_noext + "_" + str_buffer + "." + ext;
		OutputIntermediateModel(filename);

#if defined(OUTPUT_TRAINER_LOG)
		EndLog();
		WriteLog(gLogFile);
#endif
	}
}

void cNeuralNetTrainer::OutputIntermediateModel(const std::string& filename) const
{
	OutputModel(filename);
}

int cNeuralNetTrainer::GetNumLearners() const
{
	return static_cast<int>(mLearners.size());
}

void cNeuralNetTrainer::ResetLearners()
{
	for (int i = 0; i < GetNumLearners(); ++i)
	{
		mLearners[i]->Reset();
	}
}

void cNeuralNetTrainer::ResetSolvers()
{
	for (int j = 0; j < GetNetPoolSize(); ++j)
	{
		if (EnableAsyncMode())
		{
			mParamServer->ResetSolver(j);
		}
		else
		{
			auto& curr_net = mNetPool[j];
			curr_net->ResetSolver();
		}
	}
}

void cNeuralNetTrainer::UpdateParamServerInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	assert(EnableAsyncMode());
	for (int i = 0; i < GetNetPoolSize(); ++i)
	{
		mParamServer->UpdateInputOffsetScale(i, offset, scale);
	}
}

void cNeuralNetTrainer::SyncNets()
{
	assert(EnableAsyncMode());
	for (int i = 0; i < GetNetPoolSize(); ++i)
	{
		SyncNet(i);
	}
}

bool cNeuralNetTrainer::IsDone() const
{
	return mDone;
}

void cNeuralNetTrainer::SyncNet(int net_id)
{
	auto& curr_net = mNetPool[net_id];
	mParamServer->SyncNet(net_id, *curr_net);
}

#if defined(OUTPUT_TRAINER_LOG)
cNeuralNetTrainer::tLog::tLog()
{
	mBuildTupleXSamples = 0;
	mBuildTupleXTime = 0;
	mBuildTupleYSamples = 0;
	mBuildTupleYTime = 0;
	mUpdateNetSamples = 0;
	mUpdateNetTime = 0;
	mStepSamples = 0;
	mStepTime = 0;

	mBuildActorTupleXSamples = 0;
	mBuildActorTupleXTime = 0;
	mBuildActorTupleYSamples = 0;
	mBuildActorTupleYTime = 0;
	mUpdateActorNetSamples = 0;
	mUpdateActorNetTime = 0;
	mStepActorSamples = 0;
	mStepActorTime = 0;

	mLockWaitSamples = 0;
	mLockWaitTime = 0;

	mTotalExpTime = 0;
	mTotalTime = 0;

	mAsyncForwardBackSamples = 0;
	mAsyncForwardBackTime = 0;
	mAsyncUpdateNetSamples = 0;
	mAsyncUpdateNetTime = 0;

	mIters = 0;
	mAvgIterTime = 0;
}

void cNeuralNetTrainer::tLog::Write(FILE* f) const
{
	fprintf(f, "Iterations: %i\n", mIters);
	fprintf(f, "Total Exp Time: %.10fs\n", mTotalExpTime);
	fprintf(f, "Total Time: %.5fs\n", mTotalTime);
	fprintf(f, "Avg Iter Time: %.5fs\n", mAvgIterTime);
	fprintf(f, "\n");

	fprintf(f, "Build Tuple X Time: %.10fs\n", mBuildTupleXTime);
	fprintf(f, "Build Tuple X Samples: %i\n", mBuildTupleXSamples);
	fprintf(f, "Build Tuple Y Time: %.10fs\n", mBuildTupleYTime);
	fprintf(f, "Build Tuple Y Samples: %i\n", mBuildTupleYSamples);
	fprintf(f, "Update Net Time: %.10fs\n", mUpdateNetTime);
	fprintf(f, "Update Net Samples: %i\n", mUpdateNetSamples);
	fprintf(f, "Step Time: %.10fs\n", mStepTime);
	fprintf(f, "Step Samples: %i\n", mStepSamples);
	fprintf(f, "\n");

	fprintf(f, "Build Actor Tuple X Time: %.10fs\n", mBuildActorTupleXTime);
	fprintf(f, "Build Actor Tuple X Samples: %i\n", mBuildActorTupleXSamples);
	fprintf(f, "Build Actor Tuple Y Time: %.10fs\n", mBuildActorTupleYTime);
	fprintf(f, "Build Actor Tuple Y Samples: %i\n", mBuildActorTupleYSamples);
	fprintf(f, "Update Actor Net Time: %.10fs\n", mUpdateActorNetTime);
	fprintf(f, "Update Actor Net Samples: %i\n", mUpdateActorNetSamples);
	fprintf(f, "Step Actor Time: %.10fs\n", mStepActorTime);
	fprintf(f, "Step Actor Samples: %i\n", mStepActorSamples);
	fprintf(f, "\n");

	fprintf(f, "Async Forward Backward Time: %.10fs\n", mAsyncForwardBackTime);
	fprintf(f, "Step Forward Backward Samples: %i\n", mAsyncForwardBackSamples);
	fprintf(f, "Async Update Net Time: %.10fs\n", mAsyncUpdateNetTime);
	fprintf(f, "Step Update Net Samples: %i\n", mAsyncUpdateNetSamples);
	fprintf(f, "\n");

	fprintf(f, "Lock Wait Time: %.10fs\n", mLockWaitTime);
	fprintf(f, "Lock Wait Samples: %i\n", mLockWaitSamples);
	fprintf(f, "\n");
}

const cNeuralNetTrainer::tLog& cNeuralNetTrainer::GetLog() const
{
	return mLog;
}

void cNeuralNetTrainer::InitLog()
{
	mLog = tLog();
	mStartTime = std::clock();
}

void cNeuralNetTrainer::EndLog()
{
	auto end_time = std::clock();
	mLog.mTotalTime = static_cast<double>(end_time - mStartTime) / CLOCKS_PER_SEC;
	mLog.mIters = GetIter();
	mLog.mAvgIterTime = mLog.mTotalTime / mLog.mIters;
}

void cNeuralNetTrainer::WriteLog(const std::string& log_file) const
{
	FILE* f = cFileUtil::OpenFile(gLogFile, "w");
	if (f != nullptr)
	{
		mLog.Write(f);
		cFileUtil::CloseFile(f);
	}
}

#endif // OUTPUT_TRAINER_LOG
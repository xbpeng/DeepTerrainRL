#include "ACTrainer.h"
#include "util/FileUtil.h"
#include "util/Util.h"
#include "QNetTrainer.h"
#include "ACLearner.h"

std::string cACTrainer::GetCriticFilename(const std::string& actor_filename)
{
	std::string file_no_ext = cFileUtil::RemoveExtension(actor_filename);
	std::string ext = cFileUtil::GetExtension(actor_filename);
	std::string critic_file = file_no_ext + "_critic." + ext;
	return critic_file;
}

cACTrainer::cACTrainer()
{
	mActorIter = 0;
}

cACTrainer::~cACTrainer()
{
}

void cACTrainer::SetActorFiles(const std::string& actor_solver_file,
								const std::string& actor_net_file)
{
	mActorSolverFile = actor_solver_file;
	mActorNetFile = actor_net_file;
}

void cACTrainer::Init(const tParams& params)
{
	mParams = params;
	mCriticNetFile = params.mNetFile;
	mCriticSolverFile = params.mSolverFile;

	mActorIter = 0;
	BuildActor(mActorSolverFile, mActorNetFile);
	mActorBatchBuffer.clear();

	cNeuralNetTrainer::Init(params);
	InitActorProblem(mActorProb);
}

void cACTrainer::Reset()
{
	mActorIter = 0;
	BuildActor(mActorSolverFile, mActorNetFile);
	mActorBatchBuffer.clear();
	cNeuralNetTrainer::Reset();
}

int cACTrainer::AddTuple(const tExpTuple& tuple)
{
	int state_size = GetStateSize();
	assert(tuple.mStateEnd.size() == state_size);
	int t = cNeuralNetTrainer::AddTuple(tuple);
	return t;
}

int cACTrainer::GetIter() const
{
	return GetCriticIter();
}

int cACTrainer::GetCriticIter() const
{
	return mIter;
}

int cACTrainer::GetActorIter() const
{
	return mActorIter;
}

void cACTrainer::SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetCriticInputOffsetScale(offset, scale);
	SetActorInputOffsetScale(offset, scale);
}

void cACTrainer::SetCriticInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	cNeuralNetTrainer::SetInputOffsetScale(offset, scale);
}

void cACTrainer::SetActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	const auto& actor = GetActor();
	actor->SetInputOffsetScale(offset, scale);
}

void cACTrainer::SetCriticOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	cNeuralNetTrainer::SetOutputOffsetScale(offset, scale);
}

void cACTrainer::SetActorOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	const auto& actor = GetActor();
	actor->SetOutputOffsetScale(offset, scale);
}

void cACTrainer::LoadCriticModel(const std::string& model_file)
{
	cNeuralNetTrainer::LoadModel(model_file);
}

void cACTrainer::LoadCriticScale(const std::string& scale_file)
{
	cNeuralNetTrainer::LoadScale(scale_file);
}

void cACTrainer::LoadActorModel(const std::string& model_file)
{
	const auto& actor = GetActor();
	actor->LoadModel(model_file);
}

void cACTrainer::LoadActorScale(const std::string& scale_file)
{
	const auto& actor = GetActor();
	actor->LoadScale(scale_file);
}

const std::string& cACTrainer::GetNetFile() const
{
	return GetActorNetFile();
}

const std::string& cACTrainer::GetSolverFile() const
{
	return GetActorSolverFile();
}

const std::string& cACTrainer::GetActorNetFile() const
{
	return mActorNetFile;
}

const std::string& cACTrainer::GetActorSolverFile() const
{
	return mActorSolverFile;
}

const std::string& cACTrainer::GetCriticNetFile() const
{
	return mCriticNetFile;
}

const std::string& cACTrainer::GetCriticSolverFile() const
{
	return mCriticSolverFile;
}

void cACTrainer::OutputModel(const std::string& filename) const
{
	std::string critic_filename = GetCriticFilename(filename);
	OutputActor(filename);
	OutputCritic(critic_filename);
}

void cACTrainer::OutputCritic(const std::string& filename) const
{
	const auto& critic = GetCritic();
	critic->OutputModel(filename);
	printf("Critic model saved to %s\n", filename.c_str());
}

void cACTrainer::OutputActor(const std::string& filename) const
{
	const auto& actor = GetActor();
	actor->OutputModel(filename);
	printf("Actor model saved to %s\n", filename.c_str());
}

const std::unique_ptr<cNeuralNet>& cACTrainer::GetNet() const
{
	return GetActor();
}

int cACTrainer::GetCriticInputSize() const
{
	const auto& curr_net = GetCritic();
	return curr_net->GetInputSize();
}

int cACTrainer::GetCriticOutputSize() const
{
	const auto& curr_net = GetCritic();
	return curr_net->GetOutputSize();
}

int cACTrainer::GetActorInputSize() const
{
	const auto& curr_net = GetActor();
	return curr_net->GetInputSize();
}

int cACTrainer::GetActorOutputSize() const
{
	const auto& curr_net = GetActor();
	return curr_net->GetOutputSize();
}

void cACTrainer::InitActorProblem(cNeuralNet::tProblem& out_prob) const
{
	const auto& curr_net = GetActor();
	const int x_size = curr_net->GetInputSize();
	const int y_size = curr_net->GetOutputSize();
	int num_data = GetBatchSize();

	out_prob.mX.resize(num_data, x_size);
	out_prob.mY.resize(num_data, y_size);
	out_prob.mPassesPerStep = 1;
}

void cACTrainer::InitStage()
{
	if (mParams.mRewardMode == eRewardModeAvg)
	{
		InitAvgReward();
	}
	
	cNeuralNetTrainer::InitStage();
}

void cACTrainer::InitAvgReward()
{
	double avg_reward = 0;
	for (int i = 0; i < mNumTuples; ++i)
	{
		tExpTuple tuple = GetTuple(i);
		avg_reward += tuple.mReward / mNumTuples;
	}
	mAvgReward = avg_reward;
}

void cACTrainer::BuildActor(const std::string& solver_file, const std::string& net_file)
{
	mActorNet = std::unique_ptr<cNeuralNet>(new cNeuralNet());
	mActorNet->LoadNet(net_file);
	mActorNet->LoadSolver(solver_file);
}

void cACTrainer::BuildTupleActorX(const tExpTuple& tuple, Eigen::VectorXd& out_x)
{
	out_x = tuple.mStateBeg;
}

void cACTrainer::BuildTupleActorY(const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	out_y = tuple.mAction;
}

void cACTrainer::BuildActorProblemX(const std::vector<int>& tuple_ids, cNeuralNet::tProblem& out_prob)
{
	int batch_size = GetActorBatchSize();
	batch_size = std::min(batch_size, static_cast<int>(tuple_ids.size()));
	for (int i = 0; i < batch_size; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		Eigen::VectorXd x;
		BuildTupleActorX(tuple, x);
		out_prob.mX.row(i) = x;
	}
}

void cACTrainer::BuildActorProblemY(const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	int batch_size = GetActorBatchSize();
	batch_size = std::min(batch_size, static_cast<int>(tuple_ids.size()));
	for (int i = 0; i < batch_size; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		Eigen::VectorXd y;
		BuildTupleActorY(tuple, y);
		out_prob.mY.row(i) = y;
	}
}

void cACTrainer::BuildActorTupleXNext(const tExpTuple& tuple, Eigen::VectorXd& out_x)
{
	out_x = tuple.mStateEnd;
}

void cACTrainer::FetchActorMinibatch(int batch_size, std::vector<int>& out_batch)
{
	cNeuralNetTrainer::FetchMinibatch(batch_size, out_batch);
}

void cACTrainer::BuildCriticXNext(const tExpTuple& tuple, Eigen::VectorXd& out_x)
{
	out_x = tuple.mStateEnd;
}

void cACTrainer::ApplySteps(int num_steps)
{
	printf("Actor Iter %i\n", mActorIter);
	cNeuralNetTrainer::ApplySteps(num_steps);
}

void cACTrainer::UpdateMisc(const std::vector<int>& tuple_ids)
{
	if (mParams.mRewardMode == eRewardModeAvg)
	{
		UpdateAvgReward(tuple_ids);
	}
}

void cACTrainer::UpdateAvgReward(const std::vector<int>& tuple_ids)
{
	double avg_reward = 0;
	int num_data = static_cast<int>(tuple_ids.size());
	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);
		double r = tuple.mReward;

		avg_reward += r;
	}
	avg_reward /= num_data;
	
	mAvgReward += mParams.mAvgRewardStep * (avg_reward - mAvgReward);
}

void cACTrainer::UpdateCurrActiveNetID()
{
	mCurrActiveNet = (mCurrActiveNet + 1) % GetNetPoolSize();
}

void cACTrainer::UpdateOffsetScale()
{
	UpdateCriticOffsetScale();
	UpdateActorOffsetScale();
}

void cACTrainer::UpdateCriticOffsetScale()
{
	const auto& curr_net = GetCritic();
	int input_size = curr_net->GetInputSize();
	Eigen::MatrixXd X(mNumTuples, input_size);

	Eigen::VectorXd x;
	for (int i = 0; i < mNumTuples; ++i)
	{
		tExpTuple tuple = GetTuple(i);
		BuildTupleX(tuple, x);

		assert(x.size() == input_size);
		X.row(i) = x;
	}

	Eigen::VectorXd offset;
	Eigen::VectorXd scale;
	curr_net->CalcOffsetScale(X, offset, scale);
	SetCriticInputOffsetScale(offset, scale);

	if (EnableAsyncMode())
	{
		UpdateParamServerCriticInputOffsetScale(offset, scale);
	}
}

void cACTrainer::UpdateActorOffsetScale()
{
	const auto& curr_net = GetActor();
	int input_size = curr_net->GetInputSize();
	Eigen::MatrixXd X(mNumTuples, input_size);

	Eigen::VectorXd x;
	for (int i = 0; i < mNumTuples; ++i)
	{
		tExpTuple tuple = GetTuple(i);
		BuildTupleActorX(tuple, x);

		assert(x.size() == input_size);
		X.row(i) = x;
	}

	Eigen::VectorXd offset;
	Eigen::VectorXd scale;
	curr_net->CalcOffsetScale(X, offset, scale);
	SetActorInputOffsetScale(offset, scale);

	if (EnableAsyncMode())
	{
		UpdateParamServerActorInputOffsetScale(offset, scale);
	}
}

bool cACTrainer::Step()
{
	for (int i = 0; i < GetNetPoolSize(); ++i)
	{
		printf("Update Net %i:\n", i);
		bool succ = BuildProblem(i, mProb);
		if (succ)
		{
			UpdateNet(i, mProb);
		}
	}

	UpdateActor();

	return true;
}

int cACTrainer::CalcBufferSize() const
{
	return 1 + GetStateSize() * 2 + GetActionSize();
}

int cACTrainer::GetRewardIdx() const
{
	return 0;
}

int cACTrainer::GetStateBegIdx() const
{
	return 1;
}

int cACTrainer::GetStateEndIdx() const
{
	return 1 + GetStateSize() + GetActionSize();
}

int cACTrainer::GetStateSize() const
{
	int size = 0;
	const auto& curr_net = GetActor();
	if (curr_net->HasNet())
	{
		size = curr_net->GetInputSize();
	}
	return size;
}

int cACTrainer::GetActionIdx() const
{
	return 1 + GetStateSize();
}

int cACTrainer::GetActionSize() const
{
	int size = 0;
	const auto& curr_net = GetActor();
	if (curr_net->HasNet())
	{
		size = curr_net->GetOutputSize();
	}
	return size;
}

void cACTrainer::SetTuple(int t, const tExpTuple& tuple)
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

tExpTuple cACTrainer::GetTuple(int t) const
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

int cACTrainer::GetActorBatchSize() const
{
	const std::unique_ptr<cNeuralNet>& net = GetActor();
	return net->GetBatchSize();
}

const std::unique_ptr<cNeuralNet>& cACTrainer::GetCritic() const
{
	return cNeuralNetTrainer::GetCurrNet();
}

const std::unique_ptr<cNeuralNet>& cACTrainer::GetActor() const
{
	return mActorNet;
}

bool cACTrainer::HasInitModel() const
{
	return HasActorInitModel() && HasCriticInitModel();
}

bool cACTrainer::HasActorInitModel() const
{
	bool has_init_model = false;
	const auto& actor = GetActor();
	if (actor != nullptr)
	{
		has_init_model = actor->HasValidModel();
	}
	return has_init_model;
}

bool cACTrainer::HasCriticInitModel() const
{
	bool has_init_model = false;
	const auto& critic = GetCritic();
	if (critic != nullptr)
	{
		has_init_model = critic->HasValidModel();
	}
	return has_init_model;
}

void cACTrainer::EvalNet(const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	EvalActor(tuple, out_y);
}

void cACTrainer::EvalActor(const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	Eigen::VectorXd x;
	BuildTupleActorX(tuple, x);
	const auto& net = GetActor();
	net->Eval(x, out_y);
}

void cACTrainer::EvalCritic(const tExpTuple& tuple, Eigen::VectorXd& out_y)
{
	/*
	Eigen::VectorXd x;
	BuildTupleX(tuple, x);
	const auto& net = GetCritic();
	net->Eval(x, out_y);
	*/

	Eigen::VectorXd x;
	Eigen::VectorXd y;
	BuildTupleX(tuple, x);
	out_y = Eigen::VectorXd::Zero(GetCriticOutputSize());

	int num_critics = GetNetPoolSize();
	for (int i = 0; i < num_critics; ++i)
	{
		const auto& net = mNetPool[i];
		net->Eval(x, y);
		out_y += y;
	}
	out_y /= num_critics;
}

void cACTrainer::RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner)
{
	out_learner = std::shared_ptr<cACLearner>(new cACLearner(shared_from_this()));
}

void cACTrainer::UpdateActorBatchBuffer()
{
	int batch_size = GetActorBatchSize();
	FetchActorMinibatch(batch_size, mActorBatchBuffer);
}

void cACTrainer::UpdateActor()
{
	if (mStage != eStageInit)
	{
		UpdateActorBatchBuffer();
	}

	int batch_size = GetActorBatchSize();
	int buffer_size = static_cast<int>(mActorBatchBuffer.size());
	int num_batches = buffer_size / batch_size;

	for (int b = 0; b < num_batches; ++b)
	{
		StepActor();
		UpdateActorBatchBufferPostStep(batch_size);
	}
}

void cACTrainer::StepActor()
{
#if defined(OUTPUT_TRAINER_LOG)
	TIMER_RECORD_BEG(TRAIN_STEP_ACTOR)
#endif

	BuildActorProblem(mActorProb);
	UpdateActorNet(mActorProb);
	IncActorIter();

#if defined(OUTPUT_TRAINER_LOG)
	{
		std::lock_guard<std::mutex> lock(mLogLock);
		TIMER_RECORD_END(TRAIN_STEP_ACTOR, mLog.mStepActorTime, mLog.mStepActorSamples)
	}
#endif
}

void cACTrainer::IncActorIter()
{
	if (!EnableAsyncMode())
	{
		++mActorIter;
	}
}

void cACTrainer::UpdateActorNet(const cNeuralNet::tProblem& prob)
{
#if defined(OUTPUT_TRAINER_LOG)
	TIMER_RECORD_BEG(UPDATE_ACTOR_NET)
#endif

	auto& curr_net = mActorNet;
	if (EnableAsyncMode())
	{
		int net_id = GetServerActorID();

		double loss = curr_net->ForwardBackward(prob);
		printf("Actor Net Loss: %.8f\n", loss);
		
		cParamServer::tInputInfo server_input;
		server_input.mID = net_id;
		server_input.mGradNet = curr_net.get();

		cParamServer::tOutputInfo server_output;
		server_output.mSyncNet = curr_net.get();

		mParamServer->UpdateNet(server_input, server_output);
		mActorIter = server_output.mIter;
	}
	else
	{
		curr_net->Train(prob);
	}

#if defined(OUTPUT_TRAINER_LOG)
	{
		std::lock_guard<std::mutex> lock(mLogLock);
		TIMER_RECORD_END(UPDATE_ACTOR_NET, mLog.mUpdateActorNetTime, mLog.mUpdateActorNetSamples)
	}
#endif
}

void cACTrainer::BuildActorProblem(cNeuralNet::tProblem& out_prob)
{
	int num_data = GetActorBatchSize();
	int buffer_size = static_cast<int>(mActorBatchBuffer.size());
	assert(buffer_size >= num_data);
	
#if defined(OUTPUT_TRAINER_LOG)
	TIMER_RECORD_BEG(BUILD_ACTOR_TUPLE_X)
#endif
	BuildActorProblemX(mActorBatchBuffer, out_prob);
#if defined(OUTPUT_TRAINER_LOG)
	{
		std::lock_guard<std::mutex> lock(mLogLock);
		TIMER_RECORD_END(BUILD_ACTOR_TUPLE_X, mLog.mBuildActorTupleXTime, mLog.mBuildActorTupleXSamples)
	}
#endif

#if defined(OUTPUT_TRAINER_LOG)
	TIMER_RECORD_BEG(BUILD_ACTOR_TUPLE_Y)
#endif
	BuildActorProblemY(mActorBatchBuffer, out_prob.mX, out_prob);
#if defined(OUTPUT_TRAINER_LOG)
	{
		std::lock_guard<std::mutex> lock(mLogLock);
		TIMER_RECORD_END(BUILD_ACTOR_TUPLE_Y, mLog.mBuildActorTupleYTime, mLog.mBuildActorTupleYSamples)
	}
#endif
}

void cACTrainer::UpdateActorBatchBufferPostStep(int batch_size)
{
	mActorBatchBuffer.erase(mActorBatchBuffer.begin(), mActorBatchBuffer.begin() + batch_size);
}


int cACTrainer::GetServerActorID() const
{
	return mParams.mPoolSize;
}

void cACTrainer::SyncNets()
{
	cNeuralNetTrainer::SyncNets();
	SyncActorNet();
}

void cACTrainer::SyncActorNet()
{
	int net_id = GetServerActorID();
	auto& curr_net = mActorNet;
	mParamServer->SyncNet(net_id, *curr_net);
}

void cACTrainer::ResetSolvers()
{
	cNeuralNetTrainer::ResetSolvers();
	ResetActorSolver();
}

void cACTrainer::ResetActorSolver()
{
	if (EnableAsyncMode())
	{
		mParamServer->ResetSolver(GetServerActorID());
	}
	else
	{
		mActorNet->ResetSolver();
	}
}

void cACTrainer::OutputIntermediateModel(const std::string& filename) const
{
	std::string critic_filename = GetCriticFilename(filename);
	OutputActor(filename);
	OutputCritic(critic_filename);
}

void cACTrainer::UpdateParamServerCriticInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	assert(EnableAsyncMode());
	for (int i = 0; i < GetNetPoolSize(); ++i)
	{
		mParamServer->UpdateInputOffsetScale(i, offset, scale);
	}
}

void cACTrainer::UpdateParamServerActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	assert(EnableAsyncMode());
	mParamServer->UpdateInputOffsetScale(GetServerActorID(), offset, scale);
}
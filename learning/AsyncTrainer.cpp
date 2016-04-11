#include "AsyncTrainer.h"
#include "util/FileUtil.h"

const std::string gLogFile = "output/logs/async_trainer_log.txt";

cAsyncTrainer::cAsyncTrainer()
{
	mDone = false;
}

cAsyncTrainer::~cAsyncTrainer()
{
}

void cAsyncTrainer::Init(const tParams& params)
{
	mParams = params;
	mDone = false;
	cParamServer::Init();
	
	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		mTrainers[i]->Init(mParams);
	}
}

void cAsyncTrainer::Reset()
{
	cParamServer::Reset();
	mDone = false;

	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		mTrainers[i]->Reset();
	}
}

void cAsyncTrainer::Train()
{
	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		mTrainers[i]->Train();
	}
}

int cAsyncTrainer::GetIter() const
{
	int iter = 0;
	if (mTrainers.size() > 0)
	{
		iter = mTrainers[0]->GetIter();
	}
	return iter;
}

void cAsyncTrainer::EndTraining()
{
	int num_trainers = GetNumTrainers();
	for (int i = 0; i < num_trainers; ++i)
	{
		mTrainers[i]->EndTraining();
	}
	mDone = true;

#if defined(OUTPUT_TRAINER_LOG)
	WriteLog(gLogFile);
#endif
}

void cAsyncTrainer::RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner)
{
	std::shared_ptr<cNeuralNetTrainer> trainer;
	BuildTrainer(trainer);
	SetupTrainer(trainer);

	trainer->Init(mParams);
	trainer->RequestLearner(out_learner);
	mTrainers.push_back(trainer);
}

void cAsyncTrainer::LoadModel(const std::string& model_file)
{
	LoadNetModels(model_file);
	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		mTrainers[i]->LoadModel(model_file);
	}
}

void cAsyncTrainer::LoadScale(const std::string& scale_file)
{
	LoadNetScale(scale_file);
	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		mTrainers[i]->LoadScale(scale_file);
	}
}


bool cAsyncTrainer::HasInitModel() const
{
	bool has_model = false;
	if (mPool.size() > 0)
	{
		has_model = mPool[0].mNet->HasValidModel();
	}
	return has_model;
}

int cAsyncTrainer::GetStateSize() const
{
	int state_size = 0;
	if (mTrainers.size() > 0)
	{
		state_size = mTrainers[0]->GetStateSize();
	}
	return state_size;
}

int cAsyncTrainer::GetActionSize() const
{
	int action_size = 0;
	if (mTrainers.size() > 0)
	{
		action_size = mTrainers[0]->GetActionSize();
	}
	return action_size;
}

int cAsyncTrainer::GetInputSize() const
{
	int input_size = 0;
	if (mPool.size() > 0)
	{
		auto& entry = mPool[0];
		auto& net = entry.mNet;
		input_size = net->GetInputSize();
	}
	return input_size;
}

int cAsyncTrainer::GetOutputSize() const
{
	int output_size = 0;
	if (mPool.size() > 0)
	{
		auto& entry = mPool[0];
		auto& net = entry.mNet;
		output_size = net->GetOutputSize();
	}
	return output_size;
}

int cAsyncTrainer::GetNumTrainers() const
{
	return static_cast<int>(mTrainers.size());
}

void cAsyncTrainer::SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetNetInputOffsetScale(offset, scale);
	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		mTrainers[i]->SetInputOffsetScale(offset, scale);
	}
}

void cAsyncTrainer::SetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetNetOutputOffsetScale(offset, scale);
	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		mTrainers[i]->SetOutputOffsetScale(offset, scale);
	}
}

void cAsyncTrainer::OutputModel(const std::string& filename) const
{
	if (mTrainers.size() > 0)
	{
		mTrainers[0]->OutputModel(filename);
	}
}

bool cAsyncTrainer::IsDone() const
{
	return mDone;
}

int cAsyncTrainer::GetNetPoolSize() const
{
	return mParams.mPoolSize;
}

void cAsyncTrainer::BuildNetPool()
{
	int num_nets = GetNetPoolSize();
	mPool.resize(num_nets);
	for (int i = 0; i < num_nets; ++i)
	{
		SetupNet(i);
	}
}

void cAsyncTrainer::SetupNet(int id)
{
	auto& net = mPool[id].mNet;
	//net->LoadNet(mParams.mNetFile);
	net->LoadSolver(mParams.mSolverFile, true);
}

void cAsyncTrainer::BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const
{
	out_trainer = std::shared_ptr<cNeuralNetTrainer>(new cNeuralNetTrainer());
}

void cAsyncTrainer::SetupTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer)
{
	out_trainer->SetParamServer(this);
}

void cAsyncTrainer::LoadNetModels(const std::string& model_file)
{
	for (int i = 0; i < mParams.mPoolSize; ++i)
	{
		mPool[i].mNet->LoadModel(model_file);
	}
}

void cAsyncTrainer::LoadNetScale(const std::string& scale_file)
{
	for (int i = 0; i < mParams.mPoolSize; ++i)
	{
		mPool[i].mNet->LoadScale(scale_file);
	}
}

void cAsyncTrainer::SetNetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	for (int i = 0; i < mParams.mPoolSize; ++i)
	{
		mPool[i].mNet->SetInputOffsetScale(offset, scale);
		mPool[i].mScaleUpdateCount = 0;
	}
}

void cAsyncTrainer::SetNetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	for (int i = 0; i < mParams.mPoolSize; ++i)
	{
		mPool[i].mNet->SetOutputOffsetScale(offset, scale);
	}
}

#if defined(OUTPUT_TRAINER_LOG)
void cAsyncTrainer::WriteLog(const std::string& log_file) const
{
	cNeuralNetTrainer::tLog comb_log = cNeuralNetTrainer::tLog();
	int num_trainers = GetNumTrainers();
	double max_time = 0;
	for (int i = 0; i < num_trainers; ++i)
	{
		const cNeuralNetTrainer::tLog curr_log = mTrainers[i]->GetLog();

		comb_log.mBuildTupleXSamples += curr_log.mBuildTupleXSamples;
		comb_log.mBuildTupleXTime += curr_log.mBuildTupleXSamples * curr_log.mBuildTupleXTime;
		comb_log.mBuildTupleYSamples += curr_log.mBuildTupleYSamples;
		comb_log.mBuildTupleYTime += curr_log.mBuildTupleYSamples * curr_log.mBuildTupleYTime;
		comb_log.mUpdateNetSamples += curr_log.mUpdateNetSamples;
		comb_log.mUpdateNetTime += curr_log.mUpdateNetSamples * curr_log.mUpdateNetTime;
		comb_log.mStepSamples += curr_log.mStepSamples;
		comb_log.mStepTime += curr_log.mStepSamples * curr_log.mStepTime;

		comb_log.mBuildActorTupleXSamples += curr_log.mBuildActorTupleXSamples;
		comb_log.mBuildActorTupleXTime += curr_log.mBuildActorTupleXSamples * curr_log.mBuildActorTupleXTime;
		comb_log.mBuildActorTupleYSamples += curr_log.mBuildActorTupleYSamples;
		comb_log.mBuildActorTupleYTime += curr_log.mBuildActorTupleYSamples * curr_log.mBuildActorTupleYTime;
		comb_log.mUpdateActorNetSamples += curr_log.mUpdateActorNetSamples;
		comb_log.mUpdateActorNetTime += curr_log.mUpdateActorNetSamples * curr_log.mUpdateActorNetTime;
		comb_log.mStepActorSamples += curr_log.mStepActorSamples;
		comb_log.mStepActorTime += curr_log.mStepActorSamples * curr_log.mStepActorTime;

		comb_log.mAsyncForwardBackSamples += curr_log.mAsyncForwardBackSamples;
		comb_log.mAsyncForwardBackTime += curr_log.mAsyncForwardBackSamples * curr_log.mAsyncForwardBackTime;
		comb_log.mAsyncUpdateNetSamples += curr_log.mAsyncUpdateNetSamples;
		comb_log.mAsyncUpdateNetTime += curr_log.mAsyncUpdateNetSamples * curr_log.mAsyncUpdateNetTime;

		comb_log.mLockWaitSamples += curr_log.mLockWaitSamples;
		comb_log.mLockWaitTime += curr_log.mLockWaitSamples * curr_log.mLockWaitTime;

		comb_log.mTotalExpTime += curr_log.mTotalExpTime;
		comb_log.mTotalTime += curr_log.mTotalTime;
		max_time = std::max(max_time, curr_log.mTotalTime);

		comb_log.mIters = std::max(comb_log.mIters, curr_log.mIters);
	}

	comb_log.mBuildTupleXTime /= comb_log.mBuildTupleXSamples;
	comb_log.mBuildTupleYTime /= comb_log.mBuildTupleYSamples;
	comb_log.mUpdateNetTime /= comb_log.mUpdateNetSamples;
	comb_log.mStepTime /= comb_log.mStepSamples;

	comb_log.mBuildActorTupleXTime /= comb_log.mBuildActorTupleXSamples;
	comb_log.mBuildActorTupleYTime /= comb_log.mBuildActorTupleYSamples;
	comb_log.mUpdateActorNetTime /= comb_log.mUpdateActorNetSamples;
	comb_log.mStepActorTime /= comb_log.mStepActorSamples;

	comb_log.mAsyncForwardBackTime /= comb_log.mAsyncForwardBackSamples;
	comb_log.mAsyncUpdateNetTime /= comb_log.mAsyncUpdateNetSamples;

	comb_log.mLockWaitTime = comb_log.mLockWaitTime / comb_log.mLockWaitSamples;
	comb_log.mAvgIterTime = max_time / comb_log.mIters;

	double async_wait_time = mLog.mLockWaitTime;
	int async_wait_samples = mLog.mLockWaitSamples;

	FILE* f = cFileUtil::OpenFile(log_file, "w");

	comb_log.Write(f);
	
	fprintf(f, "Async Max Total Time: %.10fs\n", max_time);
	fprintf(f, "Async Wait Time: %.10fs\n", async_wait_time);
	fprintf(f, "Async Wait Samples: %i\n", async_wait_samples);

	cFileUtil::CloseFile(f);
}
#endif
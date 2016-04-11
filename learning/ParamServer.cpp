#include "ParamServer.h"
#include "util/Util.h"

cParamServer::tNetEntry::tNetEntry()
{
	mNet = std::shared_ptr<cNeuralNet>(new cNeuralNet());
	mLock = std::shared_ptr<std::mutex>(new std::mutex());
	mScaleUpdateCount = 0;
	mIter = 0;
}

cParamServer::tInputInfo::tInputInfo()
{
	mID = gInvalidIdx;
	mGradNet = nullptr;
	mIncIter = true;
}

cParamServer::tOutputInfo::tOutputInfo()
{
	mIter = 0;
	mSyncNet = nullptr;
}

cParamServer::cParamServer()
{
	mTupleCount = 0;
}

cParamServer::~cParamServer()
{
}

void cParamServer::Init()
{
	mTupleCount = 0;
	BuildNetPool();

#if defined(OUTPUT_TRAINER_LOG)
	InitLog();
#endif
}

void cParamServer::Reset()
{
	mTupleCount = 0;
	BuildNetPool();
}

int cParamServer::GetIter(int id)
{
	auto& entry = mPool[id];
	LockEntry(id);
	int iter = entry.mIter;
	UnlockEntry(id);

	return iter;
}

const cNeuralNet& cParamServer::GetNet(int id) const
{
	return *mPool[id].mNet;
}

void cParamServer::UpdateNet(const tInputInfo& in_info, tOutputInfo& out_info)
{
	int id = in_info.mID;
	cNeuralNet* grad_net = in_info.mGradNet;

	auto& entry = mPool[id];
	auto& net = mPool[id].mNet;

	LockEntry(id);

	net->CopyGrad(*grad_net);
	net->StepSolver(1);

	if (in_info.mIncIter)
	{
		++entry.mIter;
	}
	out_info.mIter = entry.mIter;

	if (out_info.mSyncNet != nullptr)
	{
		out_info.mSyncNet->CopyModel(*net);
	}

	UnlockEntry(id);
}


void cParamServer::UpdateInputOffsetScale(int id, const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	auto& entry = mPool[id];
	auto& net = mPool[id].mNet;
	int& count = entry.mScaleUpdateCount;

	LockEntry(id);
	
	Eigen::VectorXd curr_offset = net->GetInputOffset();
	Eigen::VectorXd curr_scale = net->GetInputScale();
	
	curr_offset = (count * curr_offset + offset) / (count + 1);
	curr_scale = (count * curr_scale + scale) / (count + 1);
	net->SetInputOffsetScale(curr_offset, curr_scale);
	++count;

	UnlockEntry(id);
}

void cParamServer::SyncNet(int id, cNeuralNet& out_net)
{
	auto& entry = mPool[id];
	auto& net = mPool[id].mNet;

	LockEntry(id);
	out_net.CopyModel(*net);
	UnlockEntry(id);
}

void cParamServer::ResetSolver(int id)
{
	auto& entry = mPool[id];
	auto& net = mPool[id].mNet;
	net->ResetSolver();
}

void cParamServer::LockEntry(int id)
{
#if defined(OUTPUT_TRAINER_LOG)
	TIMER_RECORD_BEG(LOCK_WAIT)
#endif

	auto& entry = mPool[id];
	entry.mLock->lock();

#if defined(OUTPUT_TRAINER_LOG)
	TIMER_RECORD_END(LOCK_WAIT, mLog.mLockWaitTime, mLog.mLockWaitSamples)
#endif
}

void cParamServer::UnlockEntry(int id)
{
	auto& entry = mPool[id];
	entry.mLock->unlock();
}

#if defined(OUTPUT_TRAINER_LOG)
cParamServer::tLog::tLog()
{
	mLockWaitTime = 0;
	mLockWaitSamples = 0;
}

const cParamServer::tLog& cParamServer::GetLog() const
{
	return mLog;
}

void cParamServer::InitLog()
{
	mLog = tLog();
}
#endif
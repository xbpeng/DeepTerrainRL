#pragma once
#include <mutex>
#include "NeuralNet.h"
#include "TrainerInterface.h"

class cParamServer
{
public:
	struct tNetEntry
	{
		std::shared_ptr<cNeuralNet> mNet;
		std::shared_ptr<std::mutex> mLock;
		int mScaleUpdateCount;
		int mIter;
		tNetEntry();
	};

	struct tInputInfo
	{
		int mID;
		cNeuralNet* mGradNet;
		bool mIncIter;

		tInputInfo();
	};

	struct tOutputInfo
	{
		int mIter;
		cNeuralNet* mSyncNet;

		tOutputInfo();
	};

	~cParamServer();

	virtual void Init();
	virtual void Reset();
	virtual int GetIter(int id);

	virtual const cNeuralNet& GetNet(int id) const;
	virtual void UpdateNet(const tInputInfo& in_info, tOutputInfo& out_info);
	virtual void UpdateInputOffsetScale(int id, const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SyncNet(int id, cNeuralNet& out_net);
	virtual void ResetSolver(int id);

	virtual void LockEntry(int id);
	virtual void UnlockEntry(int id);
	
protected:
	std::vector<tNetEntry> mPool;

	int mTupleCount;
	
	cParamServer();
	virtual void BuildNetPool() = 0;
	
#if defined(OUTPUT_TRAINER_LOG)
public:
	struct tLog
	{
		double mLockWaitTime;
		int mLockWaitSamples;
		tLog();
	};

	virtual const tLog& GetLog() const;

protected:
	tLog mLog;

	virtual void InitLog();
#endif
};
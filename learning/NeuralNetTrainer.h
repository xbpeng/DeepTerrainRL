#pragma once
#include <memory>
#include <mutex>
#include "learning/TrainerInterface.h"
#include "learning/ExpTuple.h"
#include "learning/NeuralNet.h"
#include "learning/NeuralNetLearner.h"
#include "learning/ParamServer.h"

class cNeuralNetTrainer : public cTrainerInterface, 
						public std::enable_shared_from_this<cNeuralNetTrainer>
{
public:
	enum eStage
	{
		eStageInit,
		eStageTrain,
		eStageMax
	};

	static double CalcDiscountNorm(double discount);
	
	cNeuralNetTrainer();
	virtual ~cNeuralNetTrainer();

	virtual void Init(const tParams& params);
	virtual void LoadModel(const std::string& model_file);
	virtual void LoadScale(const std::string& scale_file);
	virtual void Reset();
	virtual void EndTraining();

	virtual int AddTuple(const tExpTuple& tuple);
	virtual void AddTuples(const std::vector<tExpTuple>& tuples);
	virtual void Train();

	virtual const std::unique_ptr<cNeuralNet>& GetNet() const;
	virtual double GetDiscount() const;
	virtual double GetAvgReward() const;
	virtual int GetIter() const;
	virtual double NormalizeReward(double r) const;

	virtual void SetNumInitSamples(int num);
	virtual void SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);

	virtual int GetNumInitSamples() const;
	virtual const std::string& GetNetFile() const;
	virtual const std::string& GetSolverFile() const;

	virtual eStage GetStage() const;
	virtual int GetStateSize() const;
	virtual int GetActionSize() const;
	virtual int GetInputSize() const;
	virtual int GetOutputSize() const;
	virtual int GetBatchSize() const;

	virtual int GetNumTuples() const;
	virtual void OutputModel(const std::string& filename) const;

	virtual bool HasInitModel() const;
	virtual void EvalNet(const tExpTuple& tuple, Eigen::VectorXd& out_y);

	virtual void RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner);
	virtual int RegisterLearner(cNeuralNetLearner* learner);
	virtual void UnregisterLearner(cNeuralNetLearner* learner);

	virtual bool EnableAsyncMode() const;
	virtual void Lock();
	virtual void Unlock();

	virtual void SetParamServer(cParamServer* server);
	virtual void SyncNets();

	virtual bool IsDone() const;

protected:
	eStage mStage;
	tParams mParams;
	int mIter;
	bool mDone;

	int mBufferHead;
	int mNumTuples;
	int mTotalTuples;
	Eigen::MatrixXf mPlaybackMem;
	std::vector<unsigned int> mFlagBuffer;

	cNeuralNet::tProblem mProb;
	std::vector<std::unique_ptr<cNeuralNet>> mNetPool;
	int mCurrActiveNet;
	std::vector<int> mBatchBuffer;
	double mAvgReward;

	std::mutex mLock;
	std::vector<cNeuralNetLearner*> mLearners;

	cParamServer* mParamServer;

	const std::unique_ptr<cNeuralNet>& GetCurrNet() const;

	virtual void InitPlaybackMem(int size);
	virtual void InitBatchBuffer();
	virtual void InitProblem(cNeuralNet::tProblem& out_prob) const;
	virtual int GetPlaybackMemSize() const;
	virtual void ResetParams();
	
	virtual void Pretrain();
	virtual bool Step();
	virtual bool BuildProblem(int net_id, cNeuralNet::tProblem& out_prob);
	virtual void BuildProblemX(int net_id, const std::vector<int>& tuple_ids, cNeuralNet::tProblem& out_prob);
	virtual void BuildProblemY(int net_id, const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob);
	virtual void UpdateMisc(const std::vector<int>& tuple_ids);

	virtual void BuildTupleX(const tExpTuple& tuple, Eigen::VectorXd& out_x);
	virtual void BuildTupleY(int net_id, const tExpTuple& tuple, Eigen::VectorXd& out_y);
	virtual void FetchMinibatch(int size, std::vector<int>& out_batch);

	virtual int GetTargetNetID(int net_id) const;
	virtual void UpdateCurrActiveNetID();
	virtual const std::unique_ptr<cNeuralNet>& GetTargetNet(int net_id) const;
	virtual bool CheckTuple(const tExpTuple& tuple) const;
	virtual void UpdateNet(int net_id, const cNeuralNet::tProblem& prob);

	virtual int CalcBufferSize() const;
	virtual int GetStateBegIdx() const;
	virtual int GetActionIdx() const;

	virtual void SetTuple(int t, const tExpTuple& tuple);
	virtual tExpTuple GetTuple(int t) const;

	virtual void UpdateOffsetScale();
	virtual void UpdateStage();
	virtual void InitStage();
	virtual void ApplySteps(int num_steps);
	virtual void IncIter();

	virtual int GetNetPoolSize() const;
	virtual void BuildNetPool(const std::string& net_file, const std::string& solver_file, int pool_size);
	virtual int GetPoolSize() const;

	virtual bool EnableIntOutput() const;
	virtual void OutputIntermediate();
	virtual void OutputIntermediateModel(const std::string& filename) const;

	virtual int GetNumLearners() const;
	virtual void ResetLearners();
	virtual void ResetSolvers();

	virtual void UpdateParamServerInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SyncNet(int net_id);

#if defined(OUTPUT_TRAINER_LOG)
public:
	struct tLog
	{
		int mBuildTupleXSamples;
		double mBuildTupleXTime;
		int mBuildTupleYSamples;
		double mBuildTupleYTime;
		int mUpdateNetSamples;
		double mUpdateNetTime;
		int mStepSamples;
		double mStepTime;

		int mBuildActorTupleXSamples;
		double mBuildActorTupleXTime;
		int mBuildActorTupleYSamples;
		double mBuildActorTupleYTime;
		int mUpdateActorNetSamples;
		double mUpdateActorNetTime;
		int mStepActorSamples;
		double mStepActorTime;

		int mAsyncForwardBackSamples;
		double mAsyncForwardBackTime;
		int mAsyncUpdateNetSamples;
		double mAsyncUpdateNetTime;

		int mLockWaitSamples;
		double mLockWaitTime;

		double mTotalExpTime;
		double mTotalTime;

		int mIters;
		double mAvgIterTime;

		tLog();
		void Write(FILE* f) const;
	};

	const tLog& GetLog() const;

protected:
	std::mutex mLogLock;
	tLog mLog;
	std::clock_t mStartTime;

	void InitLog();
	void EndLog();
	void WriteLog(const std::string& log_file) const;
#endif // OUTPUT_TRAINER_LOG
};
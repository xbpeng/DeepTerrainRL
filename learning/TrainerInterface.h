#pragma once
#include <string>
#include "NeuralNet.h"
#include "NeuralNetLearner.h"

//#define OUTPUT_TRAINER_LOG

class cTrainerInterface
{
public:

	enum eRewardMode
	{
		eRewardModeStart,
		eRewardModeAvg,
		eRewardModeMax
	};

	struct tParams
	{
		std::string mNetFile;
		std::string mSolverFile;
		int mPlaybackMemSize;
		int mPoolSize;
		int mNumInitSamples;
		int mNumStepsPerIter;
		int mFreezeTargetIters; // for deep q learning
		double mDiscount;
		bool mInitInputOffsetScale;

		eRewardMode mRewardMode;
		double mAvgRewardStep;

		int mIntOutputIters;
		std::string mIntOutputFile;

		tParams();
	};
	
	virtual ~cTrainerInterface();
	virtual void Init(const tParams& params) = 0;
	virtual void Train() = 0;

	virtual void SetActorFiles(const std::string& actor_solver_file,
								const std::string& actor_net_file);

	virtual int GetIter() const = 0;
	virtual int GetCriticIter() const;
	virtual int GetActorIter() const;

	virtual void EndTraining() = 0;
	virtual void RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner) = 0;

	virtual void LoadModel(const std::string& model_file) = 0;
	virtual void LoadScale(const std::string& model_file) = 0;
	virtual void LoadCriticModel(const std::string& model_file);
	virtual void LoadCriticScale(const std::string& scale_file);
	virtual void LoadActorModel(const std::string& model_file);
	virtual void LoadActorScale(const std::string& scale_file);

	virtual bool HasInitModel() const = 0;
	virtual bool HasActorInitModel() const;
	virtual bool HasCriticInitModel() const;

	virtual int GetStateSize() const = 0;
	virtual int GetActionSize() const = 0;
	virtual int GetInputSize() const = 0;
	virtual int GetOutputSize() const = 0;

	virtual int GetCriticInputSize() const;
	virtual int GetCriticOutputSize() const;
	virtual int GetActorInputSize() const;
	virtual int GetActorOutputSize() const;

	virtual void SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) = 0;
	virtual void SetCriticInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);

	virtual void SetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale) = 0;
	virtual void SetCriticOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetActorOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);

	virtual void OutputModel(const std::string& filename) const = 0;
	virtual void OutputCritic(const std::string& filename) const;
	virtual void OutputActor(const std::string& filename) const;

	virtual bool IsDone() const = 0;

protected:

	cTrainerInterface();
};
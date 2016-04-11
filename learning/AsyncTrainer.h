#pragma once

#include "learning/TrainerInterface.h"
#include "learning/NeuralNetTrainer.h"
#include "learning/ParamServer.h"

class cAsyncTrainer : public cTrainerInterface, public cParamServer
{
public:

	cAsyncTrainer();
	virtual ~cAsyncTrainer();

	virtual void Init(const tParams& params);
	virtual void Reset();
	virtual void Train();

	virtual int GetIter() const;
	virtual void EndTraining();
	virtual void RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner);

	virtual void LoadModel(const std::string& model_file);
	virtual void LoadScale(const std::string& scale_file);
	virtual bool HasInitModel() const;

	virtual int GetStateSize() const;
	virtual int GetActionSize() const;
	virtual int GetInputSize() const;
	virtual int GetOutputSize() const;

	virtual int GetNumTrainers() const;

	virtual void SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	
	virtual void OutputModel(const std::string& filename) const;
	virtual bool IsDone() const;

protected:
	tParams mParams;
	bool mDone;
	std::vector<std::shared_ptr<cNeuralNetTrainer>> mTrainers;

	virtual int GetNetPoolSize() const;
	virtual void BuildNetPool();
	virtual void SetupNet(int id);
	
	virtual void BuildTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer) const;
	virtual void SetupTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer);

	virtual void LoadNetModels(const std::string& model_file);
	virtual void LoadNetScale(const std::string& scale_file);
	virtual void SetNetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);
	virtual void SetNetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale);

#if defined(OUTPUT_TRAINER_LOG)
	virtual void WriteLog(const std::string& log_file) const;
#endif
};
#include "TrainerInterface.h"


cTrainerInterface::tParams::tParams()
{
	mNetFile = "";
	mSolverFile = "";
	mPlaybackMemSize = 100000;
	mPoolSize = 1;
	mNumInitSamples = 1024;
	mNumStepsPerIter = 1;
	mFreezeTargetIters = 0;
	mDiscount = 0.9;
	mInitInputOffsetScale = true;

	mRewardMode = eRewardModeStart;
	mAvgRewardStep = 0.01;

	mIntOutputIters = 0;
	mIntOutputFile = "";
}

cTrainerInterface::cTrainerInterface()
{
}

cTrainerInterface::~cTrainerInterface()
{
}

void cTrainerInterface::SetActorFiles(const std::string& actor_solver_file,
										const std::string& actor_net_file)
{
}

int cTrainerInterface::GetCriticIter() const
{
	return GetIter();
}

int cTrainerInterface::GetActorIter() const
{
	return GetIter();
}

void cTrainerInterface::LoadCriticModel(const std::string& model_file)
{
	LoadModel(model_file);
}

void cTrainerInterface::LoadCriticScale(const std::string& scale_file)
{
	LoadScale(scale_file);
}

void cTrainerInterface::LoadActorModel(const std::string& model_file)
{
	LoadModel(model_file);
}

void cTrainerInterface::LoadActorScale(const std::string& scale_file)
{
	LoadScale(scale_file);
}

bool cTrainerInterface::HasActorInitModel() const
{
	return HasInitModel();
}

bool cTrainerInterface::HasCriticInitModel() const
{
	return HasInitModel();
}

int cTrainerInterface::GetCriticInputSize() const
{
	return GetInputSize();
}

int cTrainerInterface::GetCriticOutputSize() const
{
	return GetOutputSize();
}

int cTrainerInterface::GetActorInputSize() const
{
	return GetInputSize();
}

int cTrainerInterface::GetActorOutputSize() const
{
	return GetOutputSize();
}

void cTrainerInterface::SetCriticInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetInputOffsetScale(offset, scale);
}

void cTrainerInterface::SetActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetInputOffsetScale(offset, scale);
}

void cTrainerInterface::SetCriticOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetOutputOffsetScale(offset, scale);
}

void cTrainerInterface::SetActorOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetOutputOffsetScale(offset, scale);
}

void cTrainerInterface::OutputCritic(const std::string& filename) const
{
	OutputModel(filename);
}

void cTrainerInterface::OutputActor(const std::string& filename) const
{
	OutputModel(filename);
}
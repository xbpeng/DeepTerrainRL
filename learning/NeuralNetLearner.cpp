#include "NeuralNetLearner.h"
#include "NeuralNetTrainer.h"

cNeuralNetLearner::cNeuralNetLearner(const std::shared_ptr<cNeuralNetTrainer>& trainer)
{
	assert(trainer != nullptr);
	mTrainer = trainer;
	mIter = 0;
	mNumTuples = 0;
	mNet = nullptr;

	mID = mTrainer->RegisterLearner(this);
}

cNeuralNetLearner::~cNeuralNetLearner()
{
	mTrainer->UnregisterLearner(this);
}

void cNeuralNetLearner::Reset()
{
	mIter = 0;
	mNumTuples = 0;
	SyncNet();
}

void cNeuralNetLearner::Init()
{
	LoadNet(mTrainer->GetNetFile());
	SyncNet();
}

void cNeuralNetLearner::Train(const std::vector<tExpTuple>& tuples)
{
	mTrainer->Lock();

	UpdateTrainer();
	mTrainer->AddTuples(tuples);
	mTrainer->Train();
	SyncNet();

	mIter = mTrainer->GetIter();
	mNumTuples = mTrainer->GetNumTuples();

	mTrainer->Unlock();
}

int cNeuralNetLearner::GetIter() const
{
	return mIter;
}

int cNeuralNetLearner::GetNumTuples() const
{
	return mNumTuples;
}

void cNeuralNetLearner::SetNet(cNeuralNet* net)
{
	assert(net != nullptr);
	mNet = net;
}

const cNeuralNet* cNeuralNetLearner::GetNet() const
{
	return mNet;
}

void cNeuralNetLearner::LoadNet(const std::string& net_file)
{
	mNet->LoadNet(net_file);
}

void cNeuralNetLearner::LoadSolver(const std::string& solver_file)
{
	mNet->LoadSolver(solver_file);
}

void cNeuralNetLearner::OutputModel(const std::string& filename) const
{
	mNet->OutputModel(filename);
	printf("Model saved to %s\n", filename.c_str());
}

void cNeuralNetLearner::SyncNet()
{
	auto& trainer_net = mTrainer->GetNet();
	mNet->CopyModel(*trainer_net);
}

bool cNeuralNetLearner::IsDone() const
{
	return mTrainer->IsDone();
}

void cNeuralNetLearner::UpdateTrainer()
{
}
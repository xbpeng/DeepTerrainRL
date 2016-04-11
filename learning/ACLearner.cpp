#include "ACLearner.h"
#include "ACTrainer.h"

cACLearner::cACLearner(const std::shared_ptr<cNeuralNetTrainer>& trainer)
	: cNeuralNetLearner(trainer)
{
	mCriticNet = nullptr;
}

cACLearner::~cACLearner()
{
}

void cACLearner::Init()
{
	auto trainer = std::static_pointer_cast<cACTrainer>(mTrainer);
	LoadActorNet(trainer->GetActorNetFile());
	LoadCriticNet(trainer->GetCriticNetFile());
	SyncNet();
}

void cACLearner::SetNet(cNeuralNet* net)
{
	SetActorNet(net);
}

const cNeuralNet* cACLearner::GetNet() const
{
	return GetActorNet();
}

void cACLearner::SetActorNet(cNeuralNet* net)
{
	assert(net != nullptr);
	mNet = net;
}

const cNeuralNet* cACLearner::GetActorNet() const
{
	return mNet;
}

void cACLearner::SetCriticNet(cNeuralNet* net)
{
	assert(net != nullptr);
	mCriticNet = net;
}

const cNeuralNet* cACLearner::GetCriticNet() const
{
	return mCriticNet;
}

void cACLearner::LoadNet(const std::string& net_file)
{
	LoadActorNet(net_file);
}

void cACLearner::LoadSolver(const std::string& solver_file)
{
	LoadActorSolver(solver_file);
}

void cACLearner::LoadCriticNet(const std::string& net_file)
{
	if (HasCriticNet())
	{
		mCriticNet->LoadNet(net_file);
	}
}

void cACLearner::LoadCriticSolver(const std::string& solver_file)
{
	if (HasCriticNet())
	{
		mCriticNet->LoadSolver(solver_file);
	}
}

void cACLearner::LoadActorNet(const std::string& net_file)
{
	mNet->LoadNet(net_file);
}

void cACLearner::LoadActorSolver(const std::string& solver_file)
{
	mNet->LoadSolver(solver_file);
}

void cACLearner::OutputModel(const std::string& filename) const
{
	std::string critic_filename = cACTrainer::GetCriticFilename(filename);
	OutputActor(filename);
	OutputCritic(critic_filename);
}

void cACLearner::OutputCritic(const std::string& filename) const
{
	if (HasCriticNet())
	{
		mCriticNet->OutputModel(filename);
		printf("Citic model saved to %s\n", filename.c_str());
	}
}

void cACLearner::OutputActor(const std::string& filename) const
{
	mNet->OutputModel(filename);
	printf("Actor model saved to %s\n", filename.c_str());
}

void cACLearner::SyncNet()
{
	auto trainer = std::static_pointer_cast<cACTrainer>(mTrainer);

	auto& actor_net = trainer->GetActor();
	mNet->CopyModel(*actor_net);

	if (HasCriticNet())
	{
		auto& critic_net = trainer->GetCritic();
		mCriticNet->CopyModel(*critic_net);
	}
}

bool cACLearner::HasCriticNet() const
{
	return mCriticNet != nullptr;
}
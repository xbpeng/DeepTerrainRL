#include "AsyncACTrainer.h"
#include "ACTrainer.h"

cAsyncACTrainer::cAsyncACTrainer()
{
}

cAsyncACTrainer::~cAsyncACTrainer()
{
}

void cAsyncACTrainer::SetActorFiles(const std::string& actor_solver_file,
									const std::string& actor_net_file)
{
	mActorNetFile = actor_net_file;
	mActorSolverFile = actor_solver_file;
}

int cAsyncACTrainer::GetIter() const
{
	return GetCriticIter();
}

int cAsyncACTrainer::GetCriticIter() const
{
	int iter = 0;
	if (mTrainers.size() > 0)
	{
		auto trainer = std::static_pointer_cast<cACTrainer>(mTrainers[0]);
		iter = trainer->GetCriticIter();
	}
	return iter;
}

int cAsyncACTrainer::GetActorIter() const
{
	int iter = 0;
	if (mTrainers.size() > 0)
	{
		auto trainer = std::static_pointer_cast<cACTrainer>(mTrainers[0]);
		iter = trainer->GetActorIter();
	}
	return iter;
}

void cAsyncACTrainer::LoadModel(const std::string& model_file)
{
	LoadCriticModel(model_file);
}

void cAsyncACTrainer::LoadCriticModel(const std::string& model_file)
{
	int critic_start = 0;
	int critic_end = 0;
	GetCriticIDs(critic_start, critic_end);

	for (int i = critic_start; i < critic_end; ++i)
	{
		mPool[i].mNet->LoadModel(model_file);
	}

	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto trainer = std::static_pointer_cast<cACTrainer>(mTrainers[i]);
		trainer->LoadCriticModel(model_file);
	}
}

void cAsyncACTrainer::LoadActorModel(const std::string& model_file)
{
	int actor_start = 0;
	int actor_end = 0;
	GetActorIDs(actor_start, actor_end);

	for (int i = actor_start; i < actor_end; ++i)
	{
		mPool[i].mNet->LoadModel(model_file);
	}

	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto trainer = std::static_pointer_cast<cACTrainer>(mTrainers[i]);
		trainer->LoadActorModel(model_file);
	}
}

void cAsyncACTrainer::LoadScale(const std::string& scale_file)
{
	LoadCriticScale(scale_file);
}

void cAsyncACTrainer::LoadCriticScale(const std::string& scale_file)
{
	int critic_start = 0;
	int critic_end = 0;
	GetCriticIDs(critic_start, critic_end);

	for (int i = critic_start; i < critic_end; ++i)
	{
		mPool[i].mNet->LoadScale(scale_file);
		mPool[i].mScaleUpdateCount = 0;
	}

	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto trainer = std::static_pointer_cast<cACTrainer>(mTrainers[i]);
		trainer->LoadCriticScale(scale_file);
	}
}

void cAsyncACTrainer::LoadActorScale(const std::string& scale_file)
{
	int actor_start = 0;
	int actor_end = 0;
	GetActorIDs(actor_start, actor_end);

	for (int i = actor_start; i < actor_end; ++i)
	{
		mPool[i].mNet->LoadScale(scale_file);
		mPool[i].mScaleUpdateCount = 0;
	}

	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto trainer = std::static_pointer_cast<cACTrainer>(mTrainers[i]);
		trainer->LoadActorScale(scale_file);
	}
}

bool cAsyncACTrainer::HasInitModel() const
{
	return HasCriticInitModel() && HasActorInitModel();
}

bool cAsyncACTrainer::HasCriticInitModel() const
{
	bool has_model = false;

	int critic_start = 0;
	int critic_end = 0;
	GetCriticIDs(critic_start, critic_end);

	if (critic_end > critic_start)
	{
		has_model = mPool[critic_start].mNet->HasValidModel();
	}

	return has_model;
}

bool cAsyncACTrainer::HasActorInitModel() const
{
	bool has_model = false;

	int actor_start = 0;
	int actor_end = 0;
	GetActorIDs(actor_start, actor_end);

	if (actor_end > actor_start)
	{
		has_model = mPool[actor_start].mNet->HasValidModel();
	}

	return has_model;
}


int cAsyncACTrainer::GetInputSize() const
{
	return GetCriticInputSize();
}

int cAsyncACTrainer::GetOutputSize() const
{
	return GetCriticOutputSize();
}

int cAsyncACTrainer::GetCriticInputSize() const
{
	int input_size = 0;
	int critic_start = 0;
	int critic_end = 0;
	GetCriticIDs(critic_start, critic_end);

	if (critic_start < critic_end)
	{
		auto& entry = mPool[critic_start];
		auto& net = entry.mNet;
		input_size = net->GetInputSize();
	}
	return input_size;
}

int cAsyncACTrainer::GetCriticOutputSize() const
{
	int output_size = 0;
	int critic_start = 0;
	int critic_end = 0;
	GetCriticIDs(critic_start, critic_end);

	if (critic_start < critic_end)
	{
		auto& entry = mPool[critic_start];
		auto& net = entry.mNet;
		output_size = net->GetOutputSize();
	}
	return output_size;
}

int cAsyncACTrainer::GetActorInputSize() const
{
	int input_size = 0;
	int actor_start = 0;
	int actor_end = 0;
	GetActorIDs(actor_start, actor_end);

	if (actor_start < actor_end)
	{
		auto& entry = mPool[actor_start];
		auto& net = entry.mNet;
		input_size = net->GetInputSize();
	}
	return input_size;
}

int cAsyncACTrainer::GetActorOutputSize() const
{
	int output_size = 0;
	int actor_start = 0;
	int actor_end = 0;
	GetActorIDs(actor_start, actor_end);

	if (actor_start < actor_end)
	{
		auto& entry = mPool[actor_start];
		auto& net = entry.mNet;
		output_size = net->GetOutputSize();
	}
	return output_size;
}

void cAsyncACTrainer::SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetCriticInputOffsetScale(offset, scale);
}

void cAsyncACTrainer::SetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetCriticOutputOffsetScale(offset, scale);
}

void cAsyncACTrainer::SetCriticInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	int critic_start = 0;
	int critic_end = 0;
	GetCriticIDs(critic_start, critic_end);

	for (int i = critic_start; i < critic_end; ++i)
	{
		mPool[i].mNet->SetInputOffsetScale(offset, scale);
		mPool[i].mScaleUpdateCount = 0;
	}

	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto trainer = std::static_pointer_cast<cACTrainer>(mTrainers[i]);
		trainer->SetCriticInputOffsetScale(offset, scale);
	}
}

void cAsyncACTrainer::SetCriticOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	int critic_start = 0;
	int critic_end = 0;
	GetCriticIDs(critic_start, critic_end);

	for (int i = critic_start; i < critic_end; ++i)
	{
		mPool[i].mNet->SetOutputOffsetScale(offset, scale);
	}

	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto trainer = std::static_pointer_cast<cACTrainer>(mTrainers[i]);
		trainer->SetCriticOutputOffsetScale(offset, scale);
	}
}

void cAsyncACTrainer::SetActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	int actor_start = 0;
	int actor_end = 0;
	GetActorIDs(actor_start, actor_end);

	for (int i = actor_start; i < actor_end; ++i)
	{
		mPool[i].mNet->SetInputOffsetScale(offset, scale);
		mPool[i].mScaleUpdateCount = 0;
	}

	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto trainer = std::static_pointer_cast<cACTrainer>(mTrainers[i]);
		trainer->SetActorInputOffsetScale(offset, scale);
	}
}

void cAsyncACTrainer::SetActorOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	int actor_start = 0;
	int actor_end = 0;
	GetActorIDs(actor_start, actor_end);

	for (int i = actor_start; i < actor_end; ++i)
	{
		mPool[i].mNet->SetOutputOffsetScale(offset, scale);
	}

	for (int i = 0; i < GetNumTrainers(); ++i)
	{
		auto trainer = std::static_pointer_cast<cACTrainer>(mTrainers[i]);
		trainer->SetActorOutputOffsetScale(offset, scale);
	}
}

void cAsyncACTrainer::GetCriticIDs(int& out_start, int& out_end) const
{
	out_start = 0;
	out_end = mParams.mPoolSize;
	assert(out_end - out_start == GetNumCritics());
}

void cAsyncACTrainer::GetActorIDs(int& out_start, int& out_end) const
{
	out_start = mParams.mPoolSize;
	out_end = static_cast<int>(mPool.size());
	assert(out_end - out_start == GetNumActors());
}

int cAsyncACTrainer::GetNetPoolSize() const
{
	return GetNumCritics() + GetNumActors();
}

void cAsyncACTrainer::SetupNet(int id)
{
	if (IsCritic(id))
	{
		SetupCriticNet(id);
	}
	else if (IsActor(id))
	{
		SetupActorNet(id);
	}
}

void cAsyncACTrainer::SetupCriticNet(int id)
{
	auto& net = mPool[id].mNet;
	//net->LoadNet(mParams.mNetFile);
	net->LoadSolver(mParams.mSolverFile, true);
}

void cAsyncACTrainer::SetupActorNet(int id)
{
	auto& net = mPool[id].mNet;
	//net->LoadNet(mActorNetFile);
	net->LoadSolver(mActorSolverFile, true);
}

void cAsyncACTrainer::SetupTrainer(std::shared_ptr<cNeuralNetTrainer>& out_trainer)
{
	cAsyncTrainer::SetupTrainer(out_trainer);
	out_trainer->SetActorFiles(mActorSolverFile, mActorNetFile);
}

int cAsyncACTrainer::IsCritic(int id) const
{
	int start = 0;
	int end = 0;
	GetCriticIDs(start, end);
	return id >= start && id < end;
}

int cAsyncACTrainer::IsActor(int id) const
{
	int start = 0;
	int end = 0;
	GetActorIDs(start, end);
	return id >= start && id < end;
}

int cAsyncACTrainer::GetNumCritics() const
{
	return mParams.mPoolSize;
}

int cAsyncACTrainer::GetNumActors() const
{
	return 1;
}
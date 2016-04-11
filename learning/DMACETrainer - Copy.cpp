#include "DMACETrainer.h"
#include "CaclaTrainer.h"
#include "DMACELearner.h"

cDMACETrainer::cDMACETrainer()
{
	mTemp = 1;
}

cDMACETrainer::~cDMACETrainer()
{
}

void cDMACETrainer::SetActorFiles(const std::string& actor_solver_file,
									const std::string& actor_net_file)
{
	mActorSolverFile = actor_solver_file;
	mActorNetFile = actor_net_file;
}

void cDMACETrainer::Init(const tParams& params)
{
	mParams = params;
	mCriticNetFile = params.mNetFile;
	mCriticSolverFile = params.mSolverFile;

	BuildActor(mActorSolverFile, mActorNetFile);
	cMACETrainer::Init(params);
	InitActorProblem(mActorProb);
}

void cDMACETrainer::SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	SetCriticInputOffsetScale(offset, scale);
	SetActorInputOffsetScale(offset, scale);
}

void cDMACETrainer::SetCriticInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	cMACETrainer::SetInputOffsetScale(offset, scale);
}

void cDMACETrainer::SetActorInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	const auto& actor = GetActor();
	actor->SetInputOffsetScale(offset, scale);
}

void cDMACETrainer::SetCriticOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	cMACETrainer::SetOutputOffsetScale(offset, scale);
}

void cDMACETrainer::SetActorOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	const auto& actor = GetActor();
	actor->SetOutputOffsetScale(offset, scale);
}

void cDMACETrainer::LoadCriticModel(const std::string& model_file)
{
	cMACETrainer::LoadModel(model_file);
}

void cDMACETrainer::LoadCriticScale(const std::string& scale_file)
{
	cMACETrainer::LoadScale(scale_file);
}

void cDMACETrainer::LoadActorModel(const std::string& model_file)
{
	const auto& actor = GetActor();
	actor->LoadModel(model_file);
}

void cDMACETrainer::LoadActorScale(const std::string& scale_file)
{
	const auto& actor = GetActor();
	actor->LoadScale(scale_file);
}

void cDMACETrainer::OutputModel(const std::string& filename) const
{
	std::string critic_filename = cCaclaTrainer::GetCriticFilename(filename);
	OutputActor(filename);
	OutputCritic(critic_filename);
}

void cDMACETrainer::OutputCritic(const std::string& filename) const
{
	const auto& critic = GetCritic();
	critic->OutputModel(filename);
	printf("Critic model saved to %s\n", filename.c_str());
}

void cDMACETrainer::OutputActor(const std::string& filename) const
{
	const auto& actor = GetActor();
	actor->OutputModel(filename);
	printf("Actor model saved to %s\n", filename.c_str());
}


const std::string& cDMACETrainer::GetNetFile() const
{
	return GetActorNetFile();
}

const std::string& cDMACETrainer::GetSolverFile() const
{
	return GetActorSolverFile();
}

const std::string& cDMACETrainer::GetActorNetFile() const
{
	return mActorNetFile;
}

const std::string& cDMACETrainer::GetActorSolverFile() const
{
	return mActorSolverFile;
}

const std::string& cDMACETrainer::GetCriticNetFile() const
{
	return mCriticNetFile;
}

const std::string& cDMACETrainer::GetCriticSolverFile() const
{
	return mCriticSolverFile;
}

const std::unique_ptr<cNeuralNet>& cDMACETrainer::GetNet() const
{
	return GetActor();
}

const std::unique_ptr<cNeuralNet>& cDMACETrainer::GetCritic() const
{
	return cMACETrainer::GetCurrNet();
}

const std::unique_ptr<cNeuralNet>& cDMACETrainer::GetActor() const
{
	return mActorNet;
}


bool cDMACETrainer::HasInitModel() const
{
	return HasActorInitModel() && HasCriticInitModel();
}

bool cDMACETrainer::HasActorInitModel() const
{
	bool has_init_model = false;
	const auto& actor = GetActor();
	if (actor != nullptr)
	{
		has_init_model = actor->HasValidModel();
	}
	return has_init_model;
}

bool cDMACETrainer::HasCriticInitModel() const
{
	bool has_init_model = false;
	const auto& critic = GetCritic();
	if (critic != nullptr)
	{
		has_init_model = critic->HasValidModel();
	}
	return has_init_model;
}

void cDMACETrainer::SetTemp(double temp)
{
	mTemp = temp;
}

void cDMACETrainer::RequestLearner(std::shared_ptr<cNeuralNetLearner>& out_learner)
{
	out_learner = std::shared_ptr<cDMACELearner>(new cDMACELearner(this));
}

int cDMACETrainer::GetCriticInputSize() const
{
	const auto& curr_net = GetCritic();
	return curr_net->GetInputSize();
}

int cDMACETrainer::GetCriticOutputSize() const
{
	const auto& curr_net = GetCritic();
	return curr_net->GetOutputSize();
}

int cDMACETrainer::GetActorInputSize() const
{
	const auto& curr_net = GetActor();
	return curr_net->GetInputSize();
}

int cDMACETrainer::GetActorOutputSize() const
{
	const auto& curr_net = GetActor();
	return curr_net->GetOutputSize();
}

void cDMACETrainer::BuildActor(const std::string& solver_file, const std::string& net_file)
{
	mActorNet = std::unique_ptr<cNeuralNet>(new cNeuralNet());
	mActorNet->LoadNet(net_file);
	mActorNet->LoadSolver(solver_file);
}

void cDMACETrainer::InitActorProblem(cNeuralNet::tProblem& out_prob) const
{
	const auto& curr_net = GetActor();
	const int x_size = curr_net->GetInputSize();
	const int y_size = curr_net->GetOutputSize();
	int num_data = GetBatchSize();

	out_prob.mX.resize(num_data, x_size);
	out_prob.mY.resize(num_data, y_size);
	out_prob.mPassesPerStep = 1;
}

void cDMACETrainer::BuildActorProblemY(int num_data, const std::vector<int>& tuple_ids, const Eigen::MatrixXd& X, cNeuralNet::tProblem& out_prob)
{
	const auto& net = GetActor();
	net->EvalBatch(X, out_prob.mY);

	int num_actors = mNumActionFrags;
	for (int i = 0; i < num_data; ++i)
	{
		int t = tuple_ids[i];
		tExpTuple tuple = GetTuple(t);

		Eigen::VectorXd frag;
		GetActionFrag(tuple.mAction, frag);

		int selected_a = GetActionFragIdx(tuple.mAction);
		Eigen::VectorXd curr_y = out_prob.mY.row(i);
		SetFragAux(frag, selected_a, curr_y);

		double sum = 0;
		double max_val = GetMaxFragValAux(curr_y);
		for (int a = 0; a < num_actors; ++a)
		{
			double val = GetVal(curr_y, a);
			val = std::exp((val - max_val) / mTemp);
			sum += val;
		}

		Eigen::VectorXd deltas(num_actors);

		for (int a = 0; a < num_actors; ++a)
		{
			double val = GetVal(curr_y, a);
			double delta = std::exp((val - max_val) / mTemp);
			delta /= sum;
			delta /= mTemp;
			val += -delta;
			SetVal(val, a, curr_y);

			deltas[a] = -delta;
		}

		double selected_val = GetVal(curr_y, selected_a);
		//selected_val += 1;
		selected_val += 1 / mTemp;
		SetVal(selected_val, selected_a, curr_y);

		out_prob.mY.row(i) = curr_y;

		//deltas[selected_a] += 1;
		deltas[selected_a] += 1 / mTemp;
		printf("Deltas: ");
		for (int a = 0; a < num_actors; ++a)
		{
			printf("%.3f\t", deltas[a]);
		}
		printf("\n");
	}
}

void cDMACETrainer::OutputIntermediateModel(const std::string& filename) const
{
	std::string critic_filename = cCaclaTrainer::GetCriticFilename(filename);
	OutputActor(filename);
	OutputCritic(critic_filename);
}
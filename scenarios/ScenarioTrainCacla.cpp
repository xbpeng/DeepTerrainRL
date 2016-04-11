#include "ScenarioTrainCacla.h"
#include "ScenarioExpCacla.h"
#include "util/FileUtil.h"
#include "sim/BaseControllerCacla.h"
#include "learning/CaclaTrainer.h"
#include "learning/AsyncCaclaTrainer.h"
#include "learning/ACLearner.h"

cScenarioTrainCacla::cScenarioTrainCacla()
{
	mTrainerParams.mPoolSize = 1;
}

cScenarioTrainCacla::~cScenarioTrainCacla()
{
}

void cScenarioTrainCacla::ParseArgs(const cArgParser& parser)
{
	cScenarioTrain::ParseArgs(parser);
	parser.ParseString("critic_solver", mCriticSolverFile);
	parser.ParseString("critic_net", mCriticNetFile);
	parser.ParseString("critic_model", mCriticModelFile);

	mActorSolverFile = mTrainerParams.mSolverFile;
	mActorNetFile = mTrainerParams.mNetFile;
	mActorModelFile = mPoliModelFile;

	mTrainerParams.mSolverFile = mCriticSolverFile;
	mTrainerParams.mNetFile = mCriticNetFile;
}

std::string cScenarioTrainCacla::GetName() const
{
	return "Train Cacla";
}

void cScenarioTrainCacla::BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer)
{
	if (mEnableAsyncMode)
	{
		auto trainer = std::shared_ptr<cAsyncCaclaTrainer>(new cAsyncCaclaTrainer());
		trainer->SetActorFiles(mActorSolverFile, mActorNetFile);
		out_trainer = trainer;
	}
	else
	{
		auto trainer = std::shared_ptr<cCaclaTrainer>(new cCaclaTrainer());
		trainer->SetActorFiles(mActorSolverFile, mActorNetFile);
		out_trainer = trainer;
	}
}

void cScenarioTrainCacla::LoadModel()
{
	if (mCriticModelFile != "")
	{
		mTrainer->LoadCriticModel(mCriticModelFile);
	}

	if (mActorModelFile != "")
	{
		mTrainer->LoadActorModel(mActorModelFile);
	}
}

void cScenarioTrainCacla::BuildExpScene(std::shared_ptr<cScenarioExp>& out_exp) const
{
	out_exp = std::shared_ptr<cScenarioExp>(new cScenarioExpCacla());
}

void cScenarioTrainCacla::SetupLearner(const std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cNeuralNetLearner>& out_learner) const
{
	cScenarioTrain::SetupLearner(character, out_learner);

	auto ac_ctrl = std::dynamic_pointer_cast<cBaseControllerCacla>(character->GetController());
	if (ac_ctrl != nullptr)
	{
		auto ac_learner = std::static_pointer_cast<cACLearner>(out_learner);
		cNeuralNet& critic = ac_ctrl->GetCritic();
		ac_learner->SetCriticNet(&critic);
	}
	else
	{
		assert(false); // controller does not support actor-critic
	}
}

void cScenarioTrainCacla::SetupTrainerOutputOffsetScale()
{
	SetupTrainerCriticOutputOffsetScale();
	SetupTrainerActorOutputOffsetScale();
}

void cScenarioTrainCacla::SetupTrainerCriticOutputOffsetScale()
{
	bool valid_init_model = mTrainer->HasCriticInitModel();
	if (!valid_init_model)
	{
		int critic_output_size = mTrainer->GetCriticOutputSize();

		const std::shared_ptr<cCharController>& ctrl = GetRefController();
		std::shared_ptr<cBaseControllerCacla> cacla_ctrl = std::dynamic_pointer_cast<cBaseControllerCacla>(ctrl);

		if (cacla_ctrl != nullptr)
		{
			Eigen::VectorXd critic_output_offset;
			Eigen::VectorXd critic_output_scale;
			cacla_ctrl->BuildCriticOutputOffsetScale(critic_output_offset, critic_output_scale);

			assert(critic_output_offset.size() == critic_output_size);
			assert(critic_output_scale.size() == critic_output_size);
			mTrainer->SetCriticOutputOffsetScale(critic_output_offset, critic_output_scale);
		}
		else
		{
			assert(false); // controller does not implement actor-critic interface
		}
	}
}

void cScenarioTrainCacla::SetupTrainerActorOutputOffsetScale()
{
	bool valid_init_model = mTrainer->HasCriticInitModel();
	if (!valid_init_model)
	{
		int actor_output_size = mTrainer->GetActorOutputSize();

		const std::shared_ptr<cCharController>& ctrl = GetRefController();
		std::shared_ptr<cBaseControllerCacla> cacla_ctrl = std::dynamic_pointer_cast<cBaseControllerCacla>(ctrl);

		if (cacla_ctrl != nullptr)
		{
			Eigen::VectorXd actor_output_offset;
			Eigen::VectorXd actor_output_scale;
			cacla_ctrl->BuildActorOutputOffsetScale(actor_output_offset, actor_output_scale);

			assert(actor_output_offset.size() == actor_output_size);
			assert(actor_output_scale.size() == actor_output_size);
			mTrainer->SetActorOutputOffsetScale(actor_output_offset, actor_output_scale);
		}
		else
		{
			assert(false); // controller does not support CACLA
		}
	}
}
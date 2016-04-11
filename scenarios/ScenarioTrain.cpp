#include "ScenarioTrain.h"
#include <thread>

#include "sim/BaseControllerCacla.h"

const double gInitCurriculumPhase = 1;

cScenarioTrain::cScenarioTrain()
{
	mTrainerParams.mPoolSize = 2; // double Q learning
	mTrainerParams.mPlaybackMemSize = 500000;
	mTrainerParams.mNumInitSamples = 200;
	mTrainerParams.mFreezeTargetIters = 0;

	mExpPoolSize = 1;
	mMaxIter = 100000;

	mExpRate = 0.1;
	mExpTemp = 0.1;
	mExpBaseRate = 0.1;
	mInitExpRate = 1;
	mInitExpTemp = 1;
	mInitExpBaseRate = 1;

	mNumAnnealIters = 1;
	mNumBaseAnnealIters = 1;

	mNumCurriculumIters = 0;
	mNumCurriculumStageIters = 0;
	mNumCurriculumInitExpScale = 1;

	mItersPerOutput = 20;
	mTimeStep = 1 / 30.0;

	mEnableAsyncMode = false;
	EnableTraining(true);
}

cScenarioTrain::~cScenarioTrain()
{
}

void cScenarioTrain::ParseArgs(const cArgParser& parser)
{
	cScenario::ParseArgs(parser);
	
	parser.ParseString("policy_model", mPoliModelFile);
	parser.ParseString("output_path", mOutputFile);
	parser.ParseInt("trainer_max_iter", mMaxIter);

	parser.ParseInt("exp_base_anneal_iters", mNumBaseAnnealIters);
	parser.ParseDouble("init_exp_rate", mInitExpRate);
	parser.ParseDouble("init_exp_temp", mInitExpTemp);
	parser.ParseDouble("init_exp_base_rate", mInitExpBaseRate);

	parser.ParseString("policy_net", mTrainerParams.mNetFile);
	parser.ParseString("policy_solver", mTrainerParams.mSolverFile);

	parser.ParseInt("trainer_replay_mem_size", mTrainerParams.mPlaybackMemSize);
	parser.ParseBool("trainer_init_input_offset_scale", mTrainerParams.mInitInputOffsetScale);
	parser.ParseInt("trainer_num_init_samples", mTrainerParams.mNumInitSamples);
	parser.ParseInt("trainer_num_steps_per_iters", mTrainerParams.mNumStepsPerIter);
	parser.ParseInt("trainer_freeze_target_iters", mTrainerParams.mFreezeTargetIters);
	parser.ParseInt("trainer_int_iter", mTrainerParams.mIntOutputIters);
	parser.ParseString("trainer_int_output", mTrainerParams.mIntOutputFile);
	parser.ParseInt("trainer_num_anneal_iters", mNumAnnealIters);
	parser.ParseInt("trainer_curriculum_iters", mNumCurriculumIters);
	parser.ParseInt("trainer_curriculum_stage_iters", mNumCurriculumStageIters);
	parser.ParseDouble("trainer_curriculum_init_exp_scale", mNumCurriculumInitExpScale);
	parser.ParseInt("trainer_iters_per_output", mItersPerOutput);

	parser.ParseBool("trainer_enable_async_mode", mEnableAsyncMode);

	mArgParser = parser;
}

void cScenarioTrain::Init()
{
	cScenario::Init();
	BuildScenePool();
	InitTrainer();
	InitLearners();
	EnableTraining(true);
}

void cScenarioTrain::Reset()
{
	cScenario::Reset();
	ResetScenePool();
	ResetLearners();
	EnableTraining(true);
}

void cScenarioTrain::Clear()
{
	cScenario::Clear();
	mLearners.clear();
}

void cScenarioTrain::Run()
{
	int num_threads = GetPoolSize();
	std::vector<std::thread> threads(num_threads);

	for (int i = 0; i < num_threads; ++i)
	{
		std::thread& curr_thread = threads[i];
		curr_thread = std::thread(&cScenarioTrain::ExpHelper, this, mExpPool[i], i);
	}
	
	for (int i = 0; i < num_threads; ++i)
	{
		threads[i].join();
	}
}

void cScenarioTrain::Update(double time_elapsed)
{
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		UpdateExpScene(time_elapsed, *mExpPool[i].get(), i);
	}
}

void cScenarioTrain::SetExpPoolSize(int size)
{
	mExpPoolSize = size;
	mExpPoolSize = std::max(1, mExpPoolSize);
}

int cScenarioTrain::GetPoolSize() const
{
	return static_cast<int>(mExpPool.size());
}

const std::shared_ptr<cScenarioExp>& cScenarioTrain::GetExpScene(int i) const
{
	return mExpPool[i];
}

bool cScenarioTrain::TrainingComplete() const
{
	return GetIter() >= mMaxIter;
}

void cScenarioTrain::EnableTraining(bool enable)
{
	mEnableTraining = enable;

	for (int i = 0; i < GetPoolSize(); ++i)
	{
		mExpPool[i]->EnableExplore(enable);
	}
}

void cScenarioTrain::ToggleTraining()
{
	EnableTraining(!mEnableTraining);
}

bool cScenarioTrain::TrainingEnabled() const
{
	return mEnableTraining;
}

int cScenarioTrain::GetIter() const
{
	int iter = mTrainer->GetIter();
	return iter;
}

void cScenarioTrain::SetTimeStep(double time_step)
{
	mTimeStep = time_step;
}

bool cScenarioTrain::IsDone() const
{
	bool done = mTrainer->IsDone();
	int iter = GetIter();
	done |= iter >= mMaxIter;
	return done;
}

void cScenarioTrain::Shutdown()
{
	cScenario::Shutdown();
	mTrainer->EndTraining();
	mTrainer->OutputModel(mOutputFile);
}

std::string cScenarioTrain::GetName() const
{
	return "Train";
}

void cScenarioTrain::BuildScenePool()
{
	ClearScenePool();

	mExpPool.resize(mExpPoolSize);
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		auto& curr_exp = mExpPool[i];
		BuildExpScene(curr_exp);
		curr_exp->ParseArgs(mArgParser);
		curr_exp->Init();

		if (i == 0)
		{
			mExpRate = curr_exp->GetExpRate();
			mExpTemp = curr_exp->GetExpTemp();
			mExpBaseRate = curr_exp->GetExpBaseActionRate();
		}

		curr_exp->SetExpRate(mInitExpRate);
		curr_exp->SetExpTemp(mInitExpTemp);
		curr_exp->SetExpBaseActionRate(mInitExpBaseRate);
		UpdateSceneCurriculum(gInitCurriculumPhase, *curr_exp.get());
		curr_exp->Reset(); // rebuild ground
	}
}

void cScenarioTrain::ClearScenePool()
{
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		mExpPool[i]->Clear();
	}
}

void cScenarioTrain::ResetScenePool()
{
	for (int i = 0; i < GetPoolSize(); ++i)
	{
		mExpPool[i]->Reset();
	}
}

void cScenarioTrain::ResetLearners()
{
	for (size_t i = 0; i < mLearners.size(); ++i)
	{
		mLearners[i]->Reset();
	}
}

void cScenarioTrain::BuildExpScene(std::shared_ptr<cScenarioExp>& out_exp) const
{
	out_exp = std::shared_ptr<cScenarioExp>(new cScenarioExp());
}

void cScenarioTrain::InitTrainer()
{
	BuildTrainer(mTrainer);
	mTrainer->Init(mTrainerParams);
	LoadModel();
	SetupTrainerOutputOffsetScale();
}

void cScenarioTrain::InitLearners()
{
	int pool_size = GetPoolSize();
	mLearners.resize(pool_size);
	for (int i = 0; i < pool_size; ++i)
	{
		auto& curr_learner = mLearners[i];
		mTrainer->RequestLearner(curr_learner);
		auto& exp_scene = GetExpScene(i);

		const auto& character = exp_scene->GetCharacter();
		SetupLearner(character, curr_learner);
		curr_learner->Init();
	}
}

void cScenarioTrain::SetupLearner(const std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cNeuralNetLearner>& out_learner) const
{
	std::shared_ptr<cNNController> ctrl = std::static_pointer_cast<cNNController>(character->GetController());
	cNeuralNet& net = ctrl->GetNet();
	out_learner->SetNet(&net);
}

void cScenarioTrain::LoadModel()
{
	if (mPoliModelFile != "")
	{
		mTrainer->LoadModel(mPoliModelFile);
	}
}

bool cScenarioTrain::IsLearnerDone(int learner_id) const
{
	const auto& learner = mLearners[learner_id];
	bool done = learner->IsDone();
	int iter = learner->GetIter();
	done |= iter >= mMaxIter;
	return done;
}

const std::shared_ptr<cCharController>& cScenarioTrain::GetRefController() const
{
	const std::shared_ptr<cScenarioExp> exp = GetExpScene(0);
	const std::shared_ptr<cSimCharacter> character = exp->GetCharacter();
	return character->GetController();
}

void cScenarioTrain::BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer)
{
	if (mEnableAsyncMode)
	{
		auto trainer = std::shared_ptr<cAsyncQNetTrainer>(new cAsyncQNetTrainer());
		out_trainer = trainer;
	}
	else
	{
		auto trainer = std::shared_ptr<cQNetTrainer>(new cQNetTrainer());
		out_trainer = trainer;
	}
}

void cScenarioTrain::SetupTrainerOutputOffsetScale()
{
	bool valid_init_model = mTrainer->HasInitModel();
	if (!valid_init_model)
	{
		Eigen::VectorXd output_offset;
		Eigen::VectorXd output_scale;
		int output_size = mTrainer->GetOutputSize();

		const auto& exp_scene = GetExpScene(0);
		const auto& character = exp_scene->GetCharacter();
		std::shared_ptr<cNNController> ctrl = std::static_pointer_cast<cNNController>(character->GetController());
		ctrl->BuildNNOutputOffsetScale(output_offset, output_scale);

		mTrainer->SetOutputOffsetScale(output_offset, output_scale);
	}
}

void cScenarioTrain::UpdateTrainer(const std::vector<tExpTuple>& tuples, int exp_id)
{
	auto& learner = mLearners[exp_id];
	learner->Train(tuples);

	int iters = learner->GetIter();

	printf("\nIter %i\n", iters);

	int num_tuples = learner->GetNumTuples();
	printf("Num Tuples: %i\n", num_tuples);

	double curriculum_phase = CalcCurriculumPhase(iters);
	printf("Curriculum Phase: %.5f\n", curriculum_phase);

	double exp_rate = CalcExpRate(iters);
	printf("Exp Rate: %.5f\n", exp_rate);

	double exp_temp = CalcExpTemp(iters);
	printf("Exp Temp: %.5f\n", exp_temp);

	double exp_base_rate = CalcExpBaseRate(iters);
	printf("Exp Base Rate: %.5f\n", exp_base_rate);
	
	if ((iters % mItersPerOutput == 0 && iters > 0) || iters == 1)
	{
		learner->OutputModel(mOutputFile);
	}
}

void cScenarioTrain::UpdateExpScene(double time_step, cScenarioExp& out_exp, int exp_id)
{
	bool dummy_flag;
	UpdateExpScene(time_step, out_exp, exp_id, dummy_flag);
}

void cScenarioTrain::UpdateExpScene(double time_step, cScenarioExp& out_exp,
									int exp_id, bool& out_done)
{
	out_done = false;
	out_exp.Update(time_step);
	
	if (time_step > 0)
	{
		bool is_full = out_exp.IsTupleBufferFull();
		if (is_full)
		{
			if (mEnableTraining)
			{
				const std::vector<tExpTuple>& tuples = out_exp.GetTuples();
				UpdateTrainer(tuples, exp_id);

				auto& learner = mLearners[exp_id];
				int iters = learner->GetIter();

				out_done = IsLearnerDone(exp_id);

				double exp_rate = CalcExpRate(iters);
				double exp_temp = CalcExpTemp(iters);
				double exp_base_rate = CalcExpBaseRate(iters);
				double curriculum_phase = CalcCurriculumPhase(iters);
				
				out_exp.SetExpRate(exp_rate);
				out_exp.SetExpTemp(exp_temp);
				out_exp.SetExpBaseActionRate(exp_base_rate);
				UpdateSceneCurriculum(curriculum_phase, out_exp);
			}
			out_exp.ResetTupleBuffer();
		}
	}
}

void cScenarioTrain::UpdateSceneCurriculum(double phase, cScenarioExp& out_exp)
{
	double terrain_lerp = phase;
	out_exp.SetTerrainParamsLerp(terrain_lerp);
}

double cScenarioTrain::CalcExpRate(int iters) const
{
	double lerp = static_cast<double>(iters) / mNumAnnealIters;
	lerp = cMathUtil::Clamp(lerp, 0.0, 1.0);
	double exp_rate = (1 - lerp) * mInitExpRate + lerp * mExpRate;
	return exp_rate;
}

double cScenarioTrain::CalcExpTemp(int iters) const
{
	double lerp = static_cast<double>(iters) / mNumAnnealIters;
	lerp = cMathUtil::Clamp(lerp, 0.0, 1.0);
	double exp_temp = (1 - lerp) * mInitExpTemp + lerp * mExpTemp;
	return exp_temp;
}

double cScenarioTrain::CalcExpBaseRate(int iters) const
{
	double lerp = static_cast<double>(iters) / mNumBaseAnnealIters;
	lerp = cMathUtil::Clamp(lerp, 0.0, 1.0);
	double exp_rate = (1 - lerp) * mInitExpBaseRate + lerp * mExpBaseRate;
	return exp_rate;
}

double cScenarioTrain::CalcCurriculumPhase(int iters) const
{
	bool enable_curiculum = EnableCurriculum();
	double lerp = (enable_curiculum) ? (static_cast<double>(iters) / mNumCurriculumIters) : 1;
	double phase = cMathUtil::Clamp(lerp, 0.0, 1.0);

	if (iters == 0)
	{
		phase = gInitCurriculumPhase;
	}

	return phase;
}

bool cScenarioTrain::EnableCurriculum() const
{
	bool enable = mNumCurriculumIters >= 1;
	return enable;
}

void cScenarioTrain::OutputModel()
{
	mTrainer->OutputModel(mOutputFile);
}

void cScenarioTrain::ExpHelper(std::shared_ptr<cScenarioExp> exp, int exp_id)
{
	bool done = false;
	while (!done)
	{
		UpdateExpScene(mTimeStep, *exp.get(), exp_id, done);
	}
	exp->Shutdown();
}

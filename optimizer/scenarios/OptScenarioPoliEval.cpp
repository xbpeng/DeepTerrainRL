#include "OptScenarioPoliEval.h"
#include <thread>
#include <chrono>

#include "util/FileUtil.h"

cOptScenarioPoliEval::tEvalParams::tEvalParams()
{
	mMaxEpisodes = std::numeric_limits<int>::max();
	mMaxCycles = std::numeric_limits<int>::max();
}

cOptScenarioPoliEval::cOptScenarioPoliEval()
{
	mTimeStep = 1 / 30.0;
	mMaxEpisodes = std::numeric_limits<int>::max();
	mMaxCycleCount = std::numeric_limits<int>::max();
	mRandSeed = 0;

	ResetRecord();
}

cOptScenarioPoliEval::~cOptScenarioPoliEval()
{
}

void cOptScenarioPoliEval::Init()
{
	cScenario::Init();
	BuildScenePool();
	ResetRecord();
}

void cOptScenarioPoliEval::ParseArgs(const cArgParser& parser)
{
	cScenario::ParseArgs(parser);
	mArgParser = parser;

	parser.ParseInt("poli_eval_max_episodes", mMaxEpisodes);
	parser.ParseInt("poli_eval_max_cycles", mMaxCycleCount);
	parser.ParseString("output_path", mOutputFile);

	int rand_seed = 0;
	parser.ParseInt("poli_eval_rand_seed", rand_seed);
	mRandSeed = static_cast<unsigned long>(rand_seed);
}

void cOptScenarioPoliEval::Reset()
{
	cScenario::Reset();

	for (int i = 0; i < GetPoolSize(); ++i)
	{
		mEvalPool[i]->Reset();
	}

	ResetRecord();
}

void cOptScenarioPoliEval::Clear()
{
	cScenario::Clear();

	for (int i = 0; i < GetPoolSize(); ++i)
	{
		mEvalPool[i]->Clear();
	}

	ResetRecord();
}

void cOptScenarioPoliEval::Run()
{
	int num_threads = GetPoolSize();
	std::vector<std::thread> threads(num_threads);

	for (int i = 0; i < num_threads; ++i)
	{
		int max_episodes = mMaxEpisodes / num_threads;
		int episode_remainder = mMaxEpisodes % num_threads;
		if (i < episode_remainder)
		{
			++max_episodes;
		}

		int max_cycles = mMaxCycleCount / num_threads;
		int cycle_remainder = mMaxCycleCount % num_threads;
		if (i < cycle_remainder)
		{
			++max_cycles;
		}

		tEvalParams eval_params;
		eval_params.mMaxEpisodes = max_episodes;
		eval_params.mMaxCycles = max_cycles;

		std::thread& curr_thread = threads[i];
		curr_thread = std::thread(&cOptScenarioPoliEval::EvalHelper, this, eval_params, mEvalPool[i]);
	}

	for (int i = 0; i < num_threads; ++i)
	{
		threads[i].join();
	}

	if (mOutputFile != "")
	{
		OutputResults(mOutputFile);
	}
}

void cOptScenarioPoliEval::SetPoolSize(int size)
{
	mPoolSize = size;
	mPoolSize = std::max(1, mPoolSize);
}

void cOptScenarioPoliEval::SetTimeStep(double time_step)
{
	mTimeStep = time_step;
}

std::string cOptScenarioPoliEval::GetName() const
{
	return "Policy Evaluation";
}

void cOptScenarioPoliEval::ResetRecord()
{
	mEpisodeCount = 0;
	mCycleCount = 0;
	mAvgDist = 0;
}

void cOptScenarioPoliEval::BuildScenePool()
{
	mEvalPool.resize(mPoolSize);

	cRand rand;
	bool valid_seed = mRandSeed != 0;
	if (valid_seed)
	{
		rand.Seed(mRandSeed);
	}
	unsigned long curr_seed = static_cast<unsigned long>(std::abs(rand.RandInt()));

	for (int i = 0; i < GetPoolSize(); ++i)
	{
		auto& curr_eval = mEvalPool[i];
		curr_eval = std::shared_ptr<cScenarioPoliEval>(new cScenarioPoliEval());
		curr_eval->ParseArgs(mArgParser);

		curr_eval->Init();

		if (valid_seed)
		{
			curr_eval->SetRandSeed(curr_seed);
			curr_eval->Reset(); // rebuild ground
			curr_seed = static_cast<unsigned long>(std::abs(rand.RandInt()));
		}

	}
}

int cOptScenarioPoliEval::GetPoolSize() const
{
	return static_cast<int>(mEvalPool.size());
}

void cOptScenarioPoliEval::EvalHelper(tEvalParams eval_params, std::shared_ptr<cScenarioPoliEval> eval)
{
	const int num_episodes_per_update = 10;

	int num_episodes = 0;
	int num_cycles = 0;
	int prev_cycles = num_cycles;

	while (num_episodes < eval_params.mMaxEpisodes
		&& num_cycles < eval_params.mMaxCycles)
	{
		eval->Update(mTimeStep);

		num_cycles = eval->GetNumCycles();
		int curr_num_episodes = eval->GetNumEpisodes();

		if (curr_num_episodes >= num_episodes_per_update
			|| curr_num_episodes + num_episodes >= eval_params.mMaxEpisodes)
		{
			double avg_dist = eval->GetAvgDist();
			int delta_cycles = num_cycles - prev_cycles;
			UpdateRecord(avg_dist, curr_num_episodes, delta_cycles);

			eval->ResetAvgDist();
			num_episodes += curr_num_episodes;
			prev_cycles = num_cycles;
		}
	}
}

void cOptScenarioPoliEval::UpdateRecord(double avg_dist, int num_episodes, int num_cycles)
{
	std::lock_guard<std::mutex> lock_update(mUpdateMutex);

	mAvgDist = cMathUtil::AddAverage(mAvgDist, mEpisodeCount, avg_dist, num_episodes);
	mEpisodeCount += num_episodes;
	mCycleCount += num_cycles;

	printf("\nEpisodes: %i\n", mEpisodeCount);
	printf("Cycles: %i\n", mCycleCount);
	printf("Avg dist: %.5f\n", mAvgDist);
}

void cOptScenarioPoliEval::OutputResults(const std::string& out_file) const
{
	std::string str = "";

	for (int i = 0; i < GetPoolSize(); ++i)
	{
		const std::vector<double>& dist_log = mEvalPool[i]->GetDistLog();
		for (size_t j = 0; j < dist_log.size(); ++j)
		{
			double curr_dist = dist_log[j];
			if (str != "")
			{
				str += ", ";
			}
			str += std::to_string(curr_dist);
		}
	}

	str += "\n";

	bool succ = cFileUtil::AppendText(str, out_file);

	if (!succ)
	{
		printf("Failed to output results to %s\n", out_file.c_str());
	}
}
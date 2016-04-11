#pragma once

#include <string>
#include <mutex>
#include "scenarios/ScenarioPoliEval.h"

class cOptScenarioPoliEval : public cScenario
{
public:
	cOptScenarioPoliEval();
	virtual ~cOptScenarioPoliEval();

	virtual void Init();
	virtual void ParseArgs(const cArgParser& parser);
	virtual void Reset();
	virtual void Clear();

	virtual void Run();
	virtual void SetPoolSize(int size);
	virtual void SetTimeStep(double time_step);

	virtual std::string GetName() const;

protected:
	struct tEvalParams
	{
		int mMaxEpisodes;
		int mMaxCycles;

		tEvalParams();
	};

	cArgParser mArgParser;
	std::vector<std::shared_ptr<cScenarioPoliEval>> mEvalPool;
	unsigned long mRandSeed;
	int mPoolSize;
	double mTimeStep;
	int mMaxEpisodes;
	int mMaxCycleCount;
	int mEpisodeCount;
	int mCycleCount;
	double mAvgDist;

	std::mutex mUpdateMutex;

	std::string mOutputFile;

	virtual void ResetRecord();
	virtual void BuildScenePool();
	virtual int GetPoolSize() const;

	virtual void EvalHelper(tEvalParams eval_params, std::shared_ptr<cScenarioPoliEval> eval);
	virtual void UpdateRecord(double avg_dist, int num_episodes, int num_cycles);

	virtual void OutputResults(const std::string& out_file) const;
};
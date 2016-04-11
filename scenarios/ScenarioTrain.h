#pragma once

#include "scenarios/Scenario.h"
#include "scenarios/ScenarioExp.h"
#include "learning/QNetTrainer.h"
#include "learning/AsyncQNetTrainer.h"
#include <mutex>

class cScenarioTrain : public cScenario
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioTrain();
	virtual ~cScenarioTrain();

	virtual void ParseArgs(const cArgParser& parser);
	virtual void Init();
	virtual void Reset();
	virtual void Clear();

	virtual void Run();

	virtual void Update(double time_elapsed);

	virtual void SetExpPoolSize(int size);
	virtual int GetPoolSize() const;
	virtual const std::shared_ptr<cScenarioExp>& GetExpScene(int i) const;

	virtual bool TrainingComplete() const;
	virtual void EnableTraining(bool enable);
	virtual void ToggleTraining();
	virtual bool TrainingEnabled() const;
	virtual int GetIter() const;

	virtual void SetTimeStep(double time_step);
	virtual bool IsDone() const;
	virtual void Shutdown();

	virtual std::string GetName() const;

protected:

	cNeuralNetTrainer::tParams mTrainerParams;
	int mMaxIter;
	int mExpPoolSize;
	bool mEnableTraining;
	bool mEnableAsyncMode;

	double mExpRate;
	double mExpTemp;
	double mExpBaseRate;
	int mNumAnnealIters;
	int mNumBaseAnnealIters;

	int mNumCurriculumIters;
	int mNumCurriculumStageIters;
	double mNumCurriculumInitExpScale;

	double mTimeStep; // used for Run()

	double mInitExpRate;
	double mInitExpTemp;
	double mInitExpBaseRate;

	std::vector<std::shared_ptr<cScenarioExp>> mExpPool;
	std::vector<std::shared_ptr<cNeuralNetLearner>> mLearners;
	std::shared_ptr<cTrainerInterface> mTrainer;
	cArgParser mArgParser;

	std::string mPoliModelFile;
	std::string mOutputFile;
	int mItersPerOutput;

	virtual void BuildScenePool();
	virtual void ClearScenePool();
	virtual void ResetScenePool();
	virtual void ResetLearners();
	
	virtual void BuildExpScene(std::shared_ptr<cScenarioExp>& out_exp) const;

	virtual void InitTrainer();
	virtual void InitLearners();
	virtual void SetupLearner(const std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cNeuralNetLearner>& out_learner) const;
	
	virtual const std::shared_ptr<cCharController>& GetRefController() const;

	virtual void BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer);
	virtual void SetupTrainerOutputOffsetScale();
	virtual void LoadModel();
	virtual bool IsLearnerDone(int learner_id) const;

	virtual void UpdateTrainer(const std::vector<tExpTuple>& tuples, int exp_id);
	virtual void UpdateExpScene(double time_step, cScenarioExp& out_exp, int exp_id);
	virtual void UpdateExpScene(double time_step, cScenarioExp& out_exp, int exp_id, bool& out_done);
	virtual void UpdateSceneCurriculum(double phase, cScenarioExp& out_exp);
	
	virtual double CalcExpRate(int iter) const;
	virtual double CalcExpTemp(int iter) const;
	virtual double CalcExpBaseRate(int iter) const;
	virtual double CalcCurriculumPhase(int iter) const;

	virtual bool EnableCurriculum() const;
	virtual void OutputModel();

	virtual void ExpHelper(std::shared_ptr<cScenarioExp> exp, int exp_id);
};
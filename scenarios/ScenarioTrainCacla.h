#pragma once

#include "scenarios/ScenarioTrain.h"

class cScenarioTrainCacla : public cScenarioTrain
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioTrainCacla();
	virtual ~cScenarioTrainCacla();

	virtual void ParseArgs(const cArgParser& parser);

	virtual std::string GetName() const;

protected:
	std::string mActorSolverFile;
	std::string mActorNetFile;
	std::string mActorModelFile;
	std::string mCriticSolverFile;
	std::string mCriticNetFile;
	std::string mCriticModelFile;

	virtual void BuildTrainer(std::shared_ptr<cTrainerInterface>& out_trainer);
	virtual void LoadModel();
	virtual void BuildExpScene(std::shared_ptr<cScenarioExp>& out_exp) const;

	virtual void SetupLearner(const std::shared_ptr<cSimCharacter>& character, std::shared_ptr<cNeuralNetLearner>& out_learner) const;
	virtual void SetupTrainerOutputOffsetScale();
	virtual void SetupTrainerCriticOutputOffsetScale();
	virtual void SetupTrainerActorOutputOffsetScale();
};
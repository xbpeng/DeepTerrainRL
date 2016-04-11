#pragma once

#include "scenarios/ScenarioSimChar.h"
#include "learning/ExpTuple.h"
#include "learning/QNetTrainer.h"
#include "sim/NNController.h"

class cScenarioExp : public cScenarioSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioExp();
	virtual ~cScenarioExp();

	virtual void ParseArgs(const cArgParser& parser);
	virtual void Init();
	virtual void Reset();
	virtual void Clear();

	virtual void Update(double time_elapsed);

	virtual void SetBufferSize(int size);
	virtual bool IsTupleBufferFull() const;
	virtual void ResetTupleBuffer();

	virtual const std::vector<tExpTuple>& GetTuples() const;

	virtual void EnableExplore(bool enable);
	virtual void SetExpRate(double rate);
	virtual void SetExpTemp(double temp);
	virtual void SetExpBaseActionRate(double rate);
	virtual double GetExpRate() const;
	virtual double GetExpTemp() const;
	virtual double GetExpBaseActionRate() const;

	virtual std::shared_ptr<const cNNController> GetNNController() const;
	virtual std::shared_ptr<cNNController> GetNNController();

	virtual std::string GetName() const;

protected:
	std::string mPoliNetFile;
	std::string mPoliModelFile;
	std::string mCriticNetFile;
	std::string mCriticModelFile;

	int mTupleBufferSize;
	int mTupleCount;
	int mCycleCount;

	bool mEnableExplore;
	double mExpRate;
	double mExpTemp;
	double mExpBaseActionRate;

	tExpTuple mCurrTuple;
	std::vector<tExpTuple> mTupleBuffer;

	virtual bool BuildController(std::shared_ptr<cCharController>& out_ctrl);
	virtual bool BuildDogControllerCacla(std::shared_ptr<cCharController>& out_ctrl) const;
	
	virtual void ResetParams();

	virtual void PostSubstepUpdate(double time_step);
	virtual bool IsNewCycle() const;
	virtual void NewCycleUpdate();

	virtual void RecordState(Eigen::VectorXd& out_state) const;
	virtual void RecordAction(Eigen::VectorXd& out_action) const;
	virtual double CalcReward() const;
	virtual bool CheckFail() const;

	virtual void ClearFlags(tExpTuple& out_tuple) const;
	virtual void RecordFlagsBeg(tExpTuple& out_tuple) const;
	virtual void RecordFlagsEnd(tExpTuple& out_tuple) const;
	virtual void RecordTuple(const tExpTuple& tuple);

	virtual bool EnableRandInitAction() const;
	virtual void CommandRandAction();
	virtual bool IsValidTuple() const;
};
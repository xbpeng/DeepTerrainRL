#pragma once

#include "scenarios/ScenarioSimChar.h"

class cScenarioPoliEval : public cScenarioSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cScenarioPoliEval();
	virtual ~cScenarioPoliEval();

	virtual void ParseArgs(const cArgParser& parser);
	virtual void Init();
	virtual void Reset();
	virtual void Clear();
	
	virtual void Update(double time_elapsed);
	
	virtual double GetAvgDist() const;
	virtual void ResetAvgDist();
	virtual int GetNumEpisodes() const;
	virtual int GetNumCycles() const;
	virtual const std::vector<double>& GetDistLog() const;

	virtual void SetRandSeed(unsigned long seed);

	virtual std::string GetName() const;

protected:
	std::string mPoliNetFile;
	std::string mPoliModelFile;
	std::string mCriticNetFile;
	std::string mCriticModelFile;
	tVector mPosStart;

	double mAvgDist;
	int mEpisodeCount;
	int mCycleCount;
	std::vector<double> mDistLog;

	// analysis stuff
	bool mRecordNNActivation;
	std::string mNNActivationOutputFile;
	std::string mNNActivationLayer;

	bool mRecordActions;
	std::string mActionOutputFile;

	bool mRecordVel;
	std::string mVelOutputFile;
	tVector mPrevCOMPos;
	double mPrevTime;

	bool mRecordActionIDState;
	std::string mActionIDStateOutputFile;

	virtual bool BuildController(std::shared_ptr<cCharController>& out_ctrl);
	virtual bool BuildDogControllerCacla(std::shared_ptr<cCharController>& out_ctrl) const;
	virtual void RecordDistTraveled();

	virtual bool IsNewCycle() const;
	virtual void PostSubstepUpdate(double time_step);
	virtual void NewCycleUpdate();

	virtual void InitNNActivation(const std::string& out_file);
	virtual bool EnableRecordNNActivation() const;
	virtual void RecordNNActivation(const std::string& layer_name, const std::string& out_file);

	virtual void InitActionRecord(const std::string& out_file) const;
	virtual bool EnableRecordActions() const;
	virtual void RecordAction(const std::string& out_file);

	virtual void InitVelRecord(const std::string& out_file) const;
	virtual bool EnableRecordVel() const;
	virtual void RecordVel(const std::string& out_file);

	virtual void InitActionIDState(const std::string& out_file) const;
	virtual bool EnableRecordActionIDState() const;
	virtual void RecordActionIDState(const std::string& out_file);

	virtual bool IsValidCycle() const;
};
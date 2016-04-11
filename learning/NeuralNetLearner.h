#pragma once

#include <memory>
#include "ExpTuple.h"
#include "NeuralNet.h"

class cNeuralNetTrainer;

struct cNeuralNetLearner
{
public:
	cNeuralNetLearner(const std::shared_ptr<cNeuralNetTrainer>& trainer);
	virtual ~cNeuralNetLearner();

	virtual void Init();
	virtual void Reset();
	virtual void Train(const std::vector<tExpTuple>& tuples);

	virtual int GetIter() const;
	virtual int GetNumTuples() const;
	virtual void SetNet(cNeuralNet* net);
	virtual const cNeuralNet* GetNet() const;
	
	virtual void LoadNet(const std::string& net_file);
	virtual void LoadSolver(const std::string& solver_file);
	virtual void OutputModel(const std::string& filename) const;

	virtual void SyncNet();
	virtual bool IsDone() const;

protected:
	std::shared_ptr<cNeuralNetTrainer> mTrainer;
	cNeuralNet* mNet;

	int mID;
	int mIter;
	int mNumTuples;

	virtual void UpdateTrainer();
};
#pragma once
#include "NNSolver.h"

template <typename tSolverType>
class cSolverAsync : public cCaffeSolver<tSolverType>
{
public:
	cSolverAsync(const caffe::SolverParameter& param)
		: cCaffeSolver<tSolverType>(param) {};
	virtual ~cSolverAsync() {};

	virtual void ApplySteps(int steps);
};

template <typename tSolverType>
void cSolverAsync<tSolverType>::ApplySteps(int steps) {
	std::vector<caffe::Blob<cNeuralNet::tNNData>*> bottom_vec;
	const int start_iter = cSolverAsync<tSolverType>::iter_;
	const int stop_iter = cSolverAsync<tSolverType>::iter_ + steps;

	while (cSolverAsync<tSolverType>::iter_ < stop_iter) {
		cSolverAsync<tSolverType>::ApplyUpdate();
		++cSolverAsync<tSolverType>::iter_;

		caffe::SolverAction::Enum request = cSolverAsync<tSolverType>::GetRequestedAction();

		// Save a snapshot if needed.
		if ((cSolverAsync<tSolverType>::param_.snapshot()
			&& cSolverAsync<tSolverType>::iter_ % cSolverAsync<tSolverType>::param_.snapshot() == 0
			&& caffe::Caffe::root_solver()) ||
			(request == caffe::SolverAction::SNAPSHOT)) {
			cSolverAsync<tSolverType>::Snapshot();
		}
		if (caffe::SolverAction::STOP == request) {
			cSolverAsync<tSolverType>::requested_early_exit_ = true;
			// Break out of training loop.
			break;
		}
	}
}

typedef cSolverAsync<caffe::SGDSolver<cNeuralNet::tNNData>> cSGDSolverAsync;
typedef cSolverAsync<caffe::NesterovSolver<cNeuralNet::tNNData>> cNesterovSolverAsync;
typedef cSolverAsync<caffe::AdaGradSolver<cNeuralNet::tNNData>> cAdaGradSolverAsync;
typedef cSolverAsync<caffe::RMSPropSolver<cNeuralNet::tNNData>> cRMSPropSolverAsync;
typedef cSolverAsync<caffe::AdaDeltaSolver<cNeuralNet::tNNData>> cAdaDeltaSolverAsync;
typedef cSolverAsync<caffe::AdamSolver<cNeuralNet::tNNData>> cAdamSolverAsync;
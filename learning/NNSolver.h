#pragma once
#include "NeuralNet.h"
#include <caffe/net.hpp>
#include <caffe/caffe.hpp>
#include <caffe/sgd_solvers.hpp>

class cNNSolver
{
public:
	static void BuildSolver(const std::string& solver_file, std::shared_ptr<cNNSolver>& out_solver);
	static void BuildSolverAsync(const std::string& solver_file, std::shared_ptr<cNNSolver>& out_solver);

	virtual ~cNNSolver();
	virtual boost::shared_ptr<caffe::Net<cNeuralNet::tNNData>> GetNet() = 0;
	virtual void ApplySteps(int steps) = 0;
	virtual cNeuralNet::tNNData ForwardBackward() = 0;

protected:
	cNNSolver();
};

template <typename tSolverType>
class cCaffeSolver : public cNNSolver, protected tSolverType
{
public:
	cCaffeSolver(const caffe::SolverParameter& param)
		: cNNSolver(),
		  tSolverType(param) {};
	virtual ~cCaffeSolver() {};
	
	virtual boost::shared_ptr<caffe::Net<cNeuralNet::tNNData>> GetNet();
	virtual void ApplySteps(int iters);
	virtual cNeuralNet::tNNData ForwardBackward();
};

template <typename tSolverType>
boost::shared_ptr<caffe::Net<cNeuralNet::tNNData>> cCaffeSolver<tSolverType>::GetNet() {
	return tSolverType::net();
}

template <typename tSolverType>
void cCaffeSolver<tSolverType>::ApplySteps(int steps) {
	tSolverType::Step(steps);
};

template <typename tSolverType>
cNeuralNet::tNNData cCaffeSolver<tSolverType>::ForwardBackward()
{
	cNeuralNet::tNNData loss = 0;
	this->GetNet()->ClearParamDiffs();
	loss = this->GetNet()->ForwardBackward();
	return loss;
};

typedef cCaffeSolver<caffe::SGDSolver<cNeuralNet::tNNData>> cSGDSolver;
typedef cCaffeSolver<caffe::NesterovSolver<cNeuralNet::tNNData>> cNesterovSolver;
typedef cCaffeSolver<caffe::AdaGradSolver<cNeuralNet::tNNData>> cAdaGradSolver;
typedef cCaffeSolver<caffe::RMSPropSolver<cNeuralNet::tNNData>> cRMSPropSolver;
typedef cCaffeSolver<caffe::AdaDeltaSolver<cNeuralNet::tNNData>> cAdaDeltaSolver;
typedef cCaffeSolver<caffe::AdamSolver<cNeuralNet::tNNData>> cAdamSolver;
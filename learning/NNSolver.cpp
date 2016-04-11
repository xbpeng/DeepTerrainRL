#include "NNSolver.h"
#include "AsyncSolver.h"

void cNNSolver::BuildSolver(const std::string& solver_file, std::shared_ptr<cNNSolver>& out_solver)
{
	caffe::SolverParameter param;
	caffe::ReadProtoFromTextFileOrDie(solver_file, &param);
	caffe::Caffe::set_mode(caffe::Caffe::CPU);
	caffe::SolverParameter_SolverType type = param.solver_type();
	cNNSolver* solver = nullptr;

	switch (type) {
	case caffe::SolverParameter_SolverType_SGD:
		solver = new cSGDSolver(param);
		break;
	case caffe::SolverParameter_SolverType_NESTEROV:
		solver = new cNesterovSolver(param);
		break;
	case caffe::SolverParameter_SolverType_ADAGRAD:
		solver = new cAdaGradSolver(param);
		break;
	case caffe::SolverParameter_SolverType_RMSPROP:
		solver = new cRMSPropSolver(param);
		break;
	case caffe::SolverParameter_SolverType_ADADELTA:
		solver = new cAdaDeltaSolver(param);
		break;
	case caffe::SolverParameter_SolverType_ADAM:
		solver = new cAdamSolver(param);
		break;
	default:
		LOG(FATAL) << "Unknown SolverType: " << type;
	}
	out_solver = std::shared_ptr<cNNSolver>(solver);
}

void cNNSolver::BuildSolverAsync(const std::string& solver_file, std::shared_ptr<cNNSolver>& out_solver)
{
	caffe::SolverParameter param;
	caffe::ReadProtoFromTextFileOrDie(solver_file, &param);
	caffe::Caffe::set_mode(caffe::Caffe::CPU);
	caffe::SolverParameter_SolverType type = param.solver_type();
	cNNSolver* solver = nullptr;

	switch (type) {
	case caffe::SolverParameter_SolverType_SGD:
		solver = new cSGDSolverAsync(param);
		break;
	case caffe::SolverParameter_SolverType_NESTEROV:
		solver = new cNesterovSolverAsync(param);
		break;
	case caffe::SolverParameter_SolverType_ADAGRAD:
		solver = new cAdaGradSolverAsync(param);
		break;
	case caffe::SolverParameter_SolverType_RMSPROP:
		solver = new cRMSPropSolverAsync(param);
		break;
	case caffe::SolverParameter_SolverType_ADADELTA:
		solver = new cAdaDeltaSolverAsync(param);
		break;
	case caffe::SolverParameter_SolverType_ADAM:
		solver = new cAdamSolverAsync(param);
		break;
	default:
		LOG(FATAL) << "Unknown SolverType: " << type;
	}
	out_solver = std::shared_ptr<cNNSolver>(solver);
}

cNNSolver::cNNSolver()
{
}

cNNSolver::~cNNSolver()
{
}
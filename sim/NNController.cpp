#include "NNController.h"

cNNController::cNNController()
{
}

cNNController::~cNNController()
{
}

int cNNController::GetPoliStateSize() const
{
	return 0;
}

int cNNController::GetPoliActionSize() const
{
	return 0;
}

int cNNController::GetNetInputSize() const
{
	return GetPoliStateSize();
}

int cNNController::GetNetOutputSize() const
{
	return GetPoliActionSize();
}

void cNNController::RecordPoliState(Eigen::VectorXd& out_state) const
{
}

void cNNController::RecordPoliAction(Eigen::VectorXd& out_action) const
{
}

double cNNController::CalcReward() const
{
	return 0;
}

bool cNNController::IsOffPolicy() const
{
	return false;
}

bool cNNController::LoadNet(const std::string& net_file)
{
	bool succ = true;
	LoadNetIntern(net_file);

	int input_size = mNet.GetInputSize();
	int output_size = mNet.GetOutputSize();
	int state_size = GetNetInputSize();
	int action_size = GetNetOutputSize();

	if (output_size != action_size)
	{
		printf("Network output dimension does not match expected output size (%i vs %i).\n", output_size, action_size);
		succ = false;
	}

	if (input_size != state_size)
	{
		printf("Network input dimension does not match expted input size (%i vs %i).\n", input_size, state_size);
		succ = false;
	}

	if (!succ)
	{
		mNet.Clear();
		assert(false);
	}

	return succ;
}

void cNNController::LoadModel(const std::string& model_file)
{
	mNet.LoadModel(model_file);
}

void cNNController::LoadScale(const std::string& scale_file)
{
	mNet.LoadScale(scale_file);
}

void cNNController::CopyNet(const cNeuralNet& net)
{
	mNet.CopyModel(net);
}

void cNNController::SaveNet(const std::string& out_file) const
{
	mNet.OutputModel(out_file);
}

void cNNController::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int output_size = GetNetOutputSize();
	out_offset = Eigen::VectorXd::Zero(output_size);
	out_scale = Eigen::VectorXd::Ones(output_size);
}

const cNeuralNet& cNNController::GetNet() const
{
	return mNet;
}

cNeuralNet& cNNController::GetNet()
{
	return mNet;
}

bool cNNController::HasNet() const
{
	return mNet.HasNet();
}

void cNNController::LoadNetIntern(const std::string& net_file)
{
	mNet.Clear();
	mNet.LoadNet(net_file);
}
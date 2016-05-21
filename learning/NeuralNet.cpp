#include "NeuralNet.h"
#include <caffe/util/hdf5.hpp>
#include <json/json.h>

#include "util/Util.h"
#include "util/FileUtil.h"
#include "util/JsonUtil.h"
#include "NNSolver.h"
#include "AsyncSolver.h"

const std::string gInputOffsetKey = "InputOffset";
const std::string gInputScaleKey = "InputScale";
const std::string gOutputOffsetKey = "OutputOffset";
const std::string gOutputScaleKey = "OutputScale";
const std::string gInputLayerName = "data";
const std::string gOutputLayerName = "output";

std::mutex cNeuralNet::gOutputLock;

cNeuralNet::tProblem::tProblem()
{
	mX.resize(0, 0);
	mY.resize(0, 0);
	mPassesPerStep = 100;
}

bool cNeuralNet::tProblem::HasData() const
{
	return mX.size() > 0;
}



template <typename Dtype>
caffe::Solver<Dtype>* GetSolver2(const caffe::SolverParameter& param) {
	caffe::SolverParameter_SolverType type = param.solver_type();

	switch (type) {
	case caffe::SolverParameter_SolverType_SGD:
		return new caffe::SGDSolver<Dtype>(param);
	case caffe::SolverParameter_SolverType_NESTEROV:
		return new caffe::NesterovSolver<Dtype>(param);
	case caffe::SolverParameter_SolverType_ADAGRAD:
		return new caffe::AdaGradSolver<Dtype>(param);
	default:
		LOG(FATAL) << "Unknown SolverType: " << type;
	}
	return (caffe::Solver<Dtype>*) NULL;
}


cNeuralNet::cNeuralNet()
{
	Clear();
	mAsync = false;
}

cNeuralNet::~cNeuralNet()
{
}

void cNeuralNet::LoadNet(const std::string& net_file)
{
	if (net_file != "")
	{
		Clear();
		mNet = std::unique_ptr<cCaffeNetWrapper>(new cCaffeNetWrapper(net_file, caffe::TEST));

		if (!ValidOffsetScale())
		{
			InitOffsetScale();
		}

		if (HasSolver())
		{
			SyncNetParams();
		}
	}
}

void cNeuralNet::LoadModel(const std::string& model_file)
{
	if (model_file != "")
	{
		if (HasNet())
		{
			mNet->CopyTrainedLayersFromHDF5(model_file);
			LoadScale(GetOffsetScaleFile(model_file));
			SyncSolverParams();

			mValidModel = true;
		}
		else if (HasSolver())
		{
			auto net = GetTrainNet();
			net->CopyTrainedLayersFromHDF5(model_file);
			LoadScale(GetOffsetScaleFile(model_file));
			SyncNetParams();

			mValidModel = true;
		}
		else
		{
			printf("Net structure has not been initialized\n");
			assert(false);
		}
	}
}

void cNeuralNet::LoadSolver(const std::string& solver_file, bool async)
{
	if (solver_file != "")
	{
		mSolverFile = solver_file;
		mAsync = async;

		if (mAsync)
		{
			cNNSolver::BuildSolverAsync(solver_file, mSolver);
		}
		else
		{
			cNNSolver::BuildSolver(solver_file, mSolver);
		}

		if (!ValidOffsetScale())
		{
			InitOffsetScale();
		}

		if (HasNet())
		{
			SyncSolverParams();
		}
	}
}

void cNeuralNet::LoadScale(const std::string& scale_file)
{
	std::ifstream f_stream(scale_file);
	Json::Reader reader;
	Json::Value root;
	bool succ = reader.parse(f_stream, root);
	f_stream.close();

	int input_size = GetInputSize();
	if (succ && !root[gInputOffsetKey].isNull())
	{
		Eigen::VectorXd offset;
		succ &= cJsonUtil::ReadVectorJson(root[gInputOffsetKey], offset);

		int offset_size = static_cast<int>(offset.size());
		if (offset_size == input_size)
		{
			mInputOffset = offset;
		}
		else
		{
			printf("Invalid input offset size, expecting %i, but got %i\n", input_size, offset_size);
			succ = false;
		}
	}

	if (succ && !root[gInputScaleKey].isNull())
	{
		Eigen::VectorXd scale;
		succ &= cJsonUtil::ReadVectorJson(root[gInputScaleKey], scale);

		int scale_size = static_cast<int>(scale.size());
		if (scale_size == input_size)
		{
			mInputScale = scale;
		}
		else
		{
			printf("Invalid input scale size, expecting %i, but got %i\n", input_size, scale_size);
			succ = false;
		}
	}

	int output_size = GetOutputSize();
	if (succ && !root[gOutputOffsetKey].isNull())
	{
		Eigen::VectorXd offset;
		succ &= cJsonUtil::ReadVectorJson(root[gOutputOffsetKey], offset);

		int offset_size = static_cast<int>(offset.size());
		if (offset_size == output_size)
		{
			mOutputOffset = offset;
		}
		else
		{
			printf("Invalid output offset size, expecting %i, but got %i\n", output_size, offset_size);
			succ = false;
		}
	}

	if (succ && !root[gOutputScaleKey].isNull())
	{
		Eigen::VectorXd scale;
		succ &= cJsonUtil::ReadVectorJson(root[gOutputScaleKey], scale);

		int scale_size = static_cast<int>(scale.size());
		if (scale_size == output_size)
		{
			mOutputScale = scale;
		}
		else
		{
			printf("Invalid output scale size, expecting %i, but got %i\n", output_size, scale_size);
			succ = false;
		}
	}
}

void cNeuralNet::Clear()
{
	mNet.reset();
	mSolver.reset();
	mValidModel = false;

	mInputOffset.resize(0);
	mInputScale.resize(0);
	mOutputOffset.resize(0);
	mOutputScale.resize(0);
}

void cNeuralNet::Train(const tProblem& prob)
{
	if (HasSolver())
	{
		LoadTrainData(prob.mX, prob.mY);

		int batch_size = GetBatchSize();
		int num_batches = static_cast<int>(prob.mX.rows()) / batch_size;
		
		StepSolver(prob.mPassesPerStep * num_batches);
	}
	else
	{
		printf("Solver has not been initialized\n");
		assert(false);
	}
}

double cNeuralNet::ForwardBackward(const tProblem& prob)
{
	double loss = 0;
	if (HasSolver())
	{
		LoadTrainData(prob.mX, prob.mY);
		loss = mSolver->ForwardBackward();
	}
	else
	{
		printf("Solver has not been initialized\n");
		assert(false);
	}
	return loss;
}

void cNeuralNet::StepSolver(int iters)
{
	mSolver->ApplySteps(iters);

	if (HasNet())
	{
		SyncNetParams();
	}
	mValidModel = true;
}

void cNeuralNet::ResetSolver()
{
	mSolver.reset();
	LoadSolver(mSolverFile, mAsync);
}

void cNeuralNet::CalcOffsetScale(const Eigen::MatrixXd& X, Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int num_pts = static_cast<int>(X.rows());
	assert(num_pts > 1);

	double norm = 1.0 / num_pts;

	const int input_size = GetInputSize();
	out_offset = Eigen::VectorXd::Zero(input_size);
	out_scale = Eigen::VectorXd::Zero(input_size);

	for (int i = 0; i < num_pts; ++i)
	{
		out_offset += norm * X.row(i);
	}

	Eigen::VectorXd curr_x = Eigen::VectorXd::Zero(num_pts);
	for (int i = 0; i < num_pts; ++i)
	{
		curr_x = X.row(i);
		curr_x -= out_offset;
		out_scale += norm * curr_x.cwiseProduct(curr_x);
	}

	out_offset = -out_offset;

	out_scale = out_scale.cwiseSqrt();
	for (int i = 0; i < out_scale.size(); ++i)
	{
		double val = out_scale[i];
		val = (val == 0) ? 0 : (1 / val);
		out_scale[i] = val;
	}
}

void cNeuralNet::SetInputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	assert(offset.size() == GetInputSize());
	assert(scale.size() == GetInputSize());
	mInputOffset = offset;
	mInputScale = scale;
}

void cNeuralNet::SetOutputOffsetScale(const Eigen::VectorXd& offset, const Eigen::VectorXd& scale)
{
	assert(offset.size() == GetOutputSize());
	assert(scale.size() == GetOutputSize());
	mOutputOffset = offset;
	mOutputScale = scale;
}

const Eigen::VectorXd& cNeuralNet::GetInputOffset() const
{
	return mInputOffset;
}

const Eigen::VectorXd& cNeuralNet::GetInputScale() const
{
	return mInputScale;
}

const Eigen::VectorXd& cNeuralNet::GetOutputOffset() const
{
	return mOutputOffset;
}

const Eigen::VectorXd& cNeuralNet::GetOutputScale() const
{
	return mOutputScale;
}


void cNeuralNet::Eval(const Eigen::VectorXd& x, Eigen::VectorXd& out_y) const
{
	const int input_size = GetInputSize();
	assert(HasNet());
	assert(x.size() == input_size);

	caffe::Blob<tNNData> blob(1, 1, 1, input_size);
	tNNData* blob_data = blob.mutable_cpu_data();
	
	Eigen::VectorXd norm_x = x;
	NormalizeInput(norm_x);

	for (int i = 0; i < blob.count(); ++i)
	{
		blob_data[i] = norm_x[i];
	}

	tNNData loss = 0;
	const std::vector<caffe::Blob<tNNData>*>& input_blobs = mNet->input_blobs();
	input_blobs[0]->CopyFrom(blob);
	const std::vector<caffe::Blob<tNNData>*>& result_arr = mNet->Forward();

	FetchOutput(result_arr, out_y);
}

void cNeuralNet::EvalBatch(const Eigen::MatrixXd& X, Eigen::MatrixXd& out_Y) const
{
	if (HasSolver() && GetBatchSize() > 1)
	{
		EvalBatchSolver(X, out_Y);
	}
	else
	{
		EvalBatchNet(X, out_Y);
	}
}

void cNeuralNet::Backward(const Eigen::VectorXd& y_diff, Eigen::VectorXd& out_x_diff) const
{
	assert(HasNet());
	int output_size = GetOutputSize();
	const auto& top_vec = mNet->top_vecs();
	const auto& top_blob = top_vec[top_vec.size() - 1][0];
	assert(y_diff.size() == output_size);
	assert(y_diff.size() == top_blob->count());
	auto top_data = top_blob->mutable_cpu_diff();

	// normalization is weird but the math seems to work out this way
	Eigen::VectorXd norm_y_diff = y_diff;
	UnnormalizeOutputDiff(norm_y_diff);

	for (int i = 0; i < output_size; ++i)
	{
		top_data[i] = norm_y_diff[i];
	}

	mNet->ClearParamDiffs();
	mNet->Backward();

	auto bottom_vecs = mNet->bottom_vecs();
	const auto& bottom_blob = bottom_vecs[0][0];
	const tNNData* bottom_data = bottom_blob->cpu_diff();

	int input_size = GetInputSize();
	assert(bottom_blob->count() == input_size);
	out_x_diff.resize(input_size);

	for (int i = 0; i < input_size; ++i)
	{
		out_x_diff[i] = bottom_data[i];
	}
	
	NormalizeInputDiff(out_x_diff);
}

void cNeuralNet::EvalBatchNet(const Eigen::MatrixXd& X, Eigen::MatrixXd& out_Y) const
{
	assert(HasNet());
	int num_data = static_cast<int>(X.rows());
	Eigen::VectorXd x;
	Eigen::VectorXd y;

	out_Y.resize(num_data, GetOutputSize());
	for (int i = 0; i < num_data; ++i)
	{
		x = X.row(i);
		Eval(x, y);
		out_Y.row(i) = y;
	}
}

void cNeuralNet::EvalBatchSolver(const Eigen::MatrixXd& X, Eigen::MatrixXd& out_Y) const
{
	assert(HasSolver());
	boost::shared_ptr<caffe::Net<cNeuralNet::tNNData>> net = GetTrainNet();

	const int num_inputs = static_cast<int>(X.rows());
	const int input_size = GetInputSize();
	const int output_size = GetOutputSize();
	assert(X.cols() == input_size);

	int batch_size = GetBatchSize();
	int num_data = static_cast<int>(X.rows());
	int num_batches = static_cast<int>(std::ceil((1.0 * X.rows()) / batch_size));
	out_Y.resize(num_data, output_size);

	std::vector<tNNData> data(batch_size * input_size);

	auto data_layer = boost::static_pointer_cast<caffe::MemoryDataLayer<tNNData>>(net->layer_by_name(GetInputLayerName()));
	
	for (int b = 0; b < num_batches; ++b)
	{
		for (int i = 0; i < batch_size; ++i)
		{
			int data_idx = b * batch_size + i;
			if (data_idx >= num_data)
			{
				break;
			}

			auto curr_data = X.row(data_idx);
			for (int j = 0; j < input_size; ++j)
			{
				double val = curr_data[j];
				if (ValidOffsetScale())
				{
					val += mInputOffset[j];
					val = val * mInputScale[j];
				}
				data[i * input_size + j] = val;
			}
		}
		data_layer->AddData(data);

		tNNData loss = 0;
		net->ForwardPrefilled(&loss);

		const std::string& output_layer_name = GetOutputLayerName();
		const auto output_blob = net->blob_by_name(output_layer_name);
		int output_blob_count = output_blob->count();
		assert(output_blob_count == batch_size * output_size);

		auto output_blob_data = output_blob->cpu_data();
		for (int i = 0; i < batch_size; ++i)
		{
			int data_idx = b * batch_size + i;
			auto curr_data = out_Y.row(data_idx);

			for (int j = 0; j < output_size; ++j)
			{
				double val = output_blob_data[i * output_size + j];
				if (ValidOffsetScale())
				{
					val /= mOutputScale[j];
					val -= mOutputOffset[j];
				}
				curr_data(j) = val;
			}
		}
	}
}


int cNeuralNet::GetInputSize() const
{
	int size = 0;
	if (HasNet())
	{
		size = mNet->input_blobs()[0]->count();
	}
	else if (HasSolver())
	{
		auto net = GetTrainNet();
		auto input_blob = net->blob_by_name(GetInputLayerName());
		size = input_blob->count();
		size /= GetBatchSize();
	}
	return size;
}

int cNeuralNet::GetOutputSize() const
{
	int size = 0;
	if (HasNet())
	{
		size = mNet->output_blobs()[0]->count();
	}
	else if (HasSolver())
	{
		auto net = GetTrainNet();
		auto output_blob = net->blob_by_name(GetOutputLayerName());
		size = output_blob->count();
		size /= GetBatchSize();
	}

	return size;
}

int cNeuralNet::GetBatchSize() const
{
	int batch_size = 0;
	if (HasSolver())
	{
		auto data_layer = GetTrainDataLayer();
		batch_size = data_layer->batch_size();
	}
	return batch_size;
}

int cNeuralNet::CalcNumParams() const
{
	int num_params = 0;
	if (HasNet())
	{
		num_params = CalcNumParams(*mNet);
	}
	return num_params;
}

void cNeuralNet::OutputModel(const std::string& out_file) const
{
	if (HasNet())
	{
		{
			// arg, hdf5 doesn't seem to be threadsafe
			std::lock_guard<std::mutex> output_lock(gOutputLock);
			mNet->ToHDF5(out_file);
		}
		std::string scale_file = GetOffsetScaleFile(out_file);
		WriteOffsetScale(scale_file);
	}
	else
	{
		printf("No valid net to output\n");
	}
}

void cNeuralNet::PrintParams() const
{
	PrintParams(*mNet);
}

void cNeuralNet::PrintParams(const caffe::Net<tNNData>& net)
{
	const auto& param_blobs = net.learnable_params();
	int num_blobs = static_cast<int>(param_blobs.size());

	for (int b = 0; b < num_blobs; ++b)
	{
		printf("Params %i:\n", b);
		auto blob = param_blobs[b];
		auto blob_data = blob->cpu_data();
		int blob_count = blob->count();
		for (int i = 0; i < blob_count; ++i)
		{
			printf("%.5f\n", blob_data[i]);
		}
	}
}

int cNeuralNet::CalcNumParams(const caffe::Net<tNNData>& net)
{
	auto layers = net.layers();
	const auto& param_blobs = net.learnable_params();
	int num_params = 0;
	int num_blobs = static_cast<int>(param_blobs.size());

	for (int b = 0; b < num_blobs; ++b)
	{
		const auto& blob = param_blobs[b];
		int count = blob->count();
		num_params += count;
	}

	return num_params;
}

void cNeuralNet::CopyModel(const caffe::Net<tNNData>& src, caffe::Net<tNNData>& dst)
{
	const auto& src_params = src.learnable_params();
	const auto& dst_params = dst.learnable_params();
	CopyParams(src_params, dst_params);
}

void cNeuralNet::CopyParams(const std::vector<caffe::Blob<tNNData>*>& src_params, const std::vector<caffe::Blob<tNNData>*>& dst_params)
{
	int num_blobs = static_cast<int>(src_params.size());
	for (int b = 0; b < num_blobs; ++b)
	{
		auto src_blob = src_params[b];
		auto dst_blob = dst_params[b];

		auto src_blob_data = src_blob->cpu_data();
		auto dst_blob_data = dst_blob->mutable_cpu_data();
		int src_blob_count = src_blob->count();
		int dst_blob_count = dst_blob->count();

		if (src_blob_count == dst_blob_count)
		{
			std::memcpy(dst_blob_data, src_blob_data, src_blob_count * sizeof(tNNData));
		}
		else
		{
			assert(false); // param size mismatch
		}
	}
}

bool cNeuralNet::CompareModel(const caffe::Net<tNNData>& a, const caffe::Net<tNNData>& b)
{
	const auto& a_params = a.learnable_params();
	const auto& b_params = b.learnable_params();
	return CompareParams(a_params, b_params);
}

bool cNeuralNet::CompareParams(const std::vector<caffe::Blob<tNNData>*>& a_params, const std::vector<caffe::Blob<tNNData>*>& b_params)
{
	int num_blobs = static_cast<int>(a_params.size());
	for (int i = 0; i < num_blobs; ++i)
	{
		auto a_blob = a_params[i];
		auto b_blob = b_params[i];

		auto a_blob_data = a_blob->cpu_data();
		auto b_blob_data = b_blob->cpu_data();
		int a_blob_count = a_blob->count();
		int b_blob_count = b_blob->count();

		if (a_blob_count == b_blob_count)
		{
			for (int j = 0; j < a_blob_count; ++j)
			{
				if (a_blob_data[j] != b_blob_data[j])
				{
					return false;
				}
			}
		}
		else
		{
			assert(false); // param size mismatch
		}
	}
	return true;
}

bool cNeuralNet::HasNet() const
{
	return mNet != nullptr;
}

bool cNeuralNet::HasSolver() const
{
	return mSolver != nullptr;
}

bool cNeuralNet::HasLayer(const std::string layer_name) const
{
	if (HasNet())
	{
		return mNet->has_blob(layer_name) && mNet->has_layer(layer_name);
	}
	return false;
}

bool cNeuralNet::HasValidModel() const
{
	return mValidModel;
}

void cNeuralNet::CopyModel(const cNeuralNet& other)
{
	CopyParams(other.GetParams(), GetParams());

	mInputOffset = other.GetInputOffset();
	mInputScale = other.GetInputScale();
	mOutputOffset = other.GetOutputOffset();
	mOutputScale = other.GetOutputScale();

	SyncSolverParams();
	mValidModel = true;
}

void cNeuralNet::LerpModel(const cNeuralNet& other, double lerp)
{
	BlendModel(other, 1 - lerp, lerp);
}

void cNeuralNet::BlendModel(const cNeuralNet& other, double this_weight, double other_weight)
{
	const auto& src_params = other.GetParams();
	const auto& dst_params = GetParams();

	int num_blobs = static_cast<int>(src_params.size());
	for (int b = 0; b < num_blobs; ++b)
	{
		auto src_blob = src_params[b];
		auto dst_blob = dst_params[b];

		auto src_blob_data = src_blob->cpu_data();
		auto dst_blob_data = dst_blob->mutable_cpu_data();
		int src_blob_count = src_blob->count();
		int dst_blob_count = dst_blob->count();
		assert(src_blob_count == dst_blob_count);

		for (int i = 0; i < src_blob_count; ++i)
		{
			dst_blob_data[i] = this_weight * dst_blob_data[i] + other_weight * src_blob_data[i];
		}
	}

	SyncSolverParams();
	mValidModel = true;
}

void cNeuralNet::BuildNetParams(caffe::NetParameter& out_params) const
{
	mNet->ToProto(&out_params);
}

bool cNeuralNet::CompareModel(const cNeuralNet& other) const
{
	bool same = CompareParams(other.GetParams(), GetParams());

	same &= mInputOffset.isApprox(other.GetInputOffset(), 0);
	same &= mInputScale.isApprox(other.GetInputScale(), 0);
	same &= mOutputOffset.isApprox(other.GetOutputOffset(), 0);
	same &= mOutputScale.isApprox(other.GetOutputScale(), 0);

	return same;
}

void cNeuralNet::ForwardInjectNoisePrefilled(double mean, double stdev, const std::string& layer_name, Eigen::VectorXd& out_y) const
{
	// assume the Eval has already been called which fills the blobs in the network using a given input
	if (HasLayer(layer_name))
	{
		auto blob = mNet->blob_by_name(layer_name);

		tNNData* data = blob->mutable_cpu_data();
		const int data_size = blob->count();

		for (int i = 0; i < data_size; ++i)
		{
			double noise = cMathUtil::RandDoubleNorm(mean, stdev);
			data[i] += noise;
		}

		int layer_idx = mNet->GetLayerIdx(layer_name);
		++layer_idx;
		mNet->ForwardFrom(layer_idx);

		const std::vector<caffe::Blob<tNNData>*>& result_arr = mNet->output_blobs();
		FetchOutput(result_arr, out_y);
	}
	else
	{
		printf("Can't find layer named %s\n", layer_name.c_str());
		assert(false); // layer not found
	}
}

void cNeuralNet::GetLayerState(const std::string& layer_name, Eigen::VectorXd& out_state) const
{
	auto blob = mNet->blob_by_name(layer_name);
	if (blob != nullptr)
	{
		const tNNData* data = blob->cpu_data();
		const int data_size = blob->count();

		out_state.resize(data_size);
		for (int i = 0; i < data_size; ++i)
		{
			out_state[i] = data[i];
		}
	}
	else
	{
		printf("Can't find layer named %s\n", layer_name.c_str());
		assert(false); // layer not found
	}
}

void cNeuralNet::SetLayerState(const Eigen::VectorXd& state, const std::string& layer_name) const
{
	auto blob = mNet->blob_by_name(layer_name);
	if (blob != nullptr)
	{
		tNNData* data = blob->mutable_cpu_data();
		const int data_size = blob->count();
		assert(state.size() == data_size);

		for (int i = 0; i < data_size; ++i)
		{
			data[i] = state[i];
		}
	}
	else
	{
		printf("Can't find layer named %s\n", layer_name.c_str());
		assert(false); // layer not found
	}
}

const std::vector<caffe::Blob<cNeuralNet::tNNData>*>& cNeuralNet::GetParams() const
{
	if (HasNet())
	{
		return mNet->learnable_params();
	}
	else
	{
		auto net = GetTrainNet();
		return net->learnable_params();
	}
}

void cNeuralNet::SyncSolverParams()
{
	if (HasSolver() && HasNet())
	{
		CopyModel(*mNet, *GetTrainNet());
	}
}

void cNeuralNet::SyncNetParams()
{
	if (HasSolver() && HasNet())
	{
		CopyModel(*GetTrainNet(), *mNet);
	}
}

void cNeuralNet::CopyGrad(const cNeuralNet& other)
{
	assert(HasSolver());
	assert(other.HasSolver());
	auto other_net = other.GetTrainNet();
	auto this_net = GetTrainNet();
	
	auto other_params = other_net->learnable_params();
	auto this_params = this_net->learnable_params();
	assert(other_params.size() == this_params.size());

	for (size_t i = 0; i < this_params.size(); ++i)
	{
		auto other_blob = other_params[i];
		auto this_blob = this_params[i];
		assert(other_blob->count() == this_blob->count());

		auto other_diff = other_blob->cpu_diff();
		auto this_diff = this_blob->mutable_cpu_diff();
		auto other_data = other_blob->cpu_data();
		auto this_data = this_blob->cpu_data();
		for (int j = 0; j < this_blob->count(); ++j)
		{
			this_diff[j] = other_diff[j];
		}
	}
}

bool cNeuralNet::ValidOffsetScale() const
{
	return mInputOffset.size() > 0 && mInputScale.size() > 0
		&& mOutputOffset.size() > 0 && mOutputScale.size() > 0;
}

void cNeuralNet::InitOffsetScale()
{
	int input_size = GetInputSize();
	mInputOffset = Eigen::VectorXd::Zero(input_size);
	mInputScale = Eigen::VectorXd::Ones(input_size);

	int output_size = GetOutputSize();
	mOutputOffset = Eigen::VectorXd::Zero(output_size);
	mOutputScale = Eigen::VectorXd::Ones(output_size);
}

void cNeuralNet::FetchOutput(const std::vector<caffe::Blob<tNNData>*>& results_arr, Eigen::VectorXd& out_y) const
{
	const caffe::Blob<tNNData>* result = results_arr[0];
	const tNNData* result_data = result->cpu_data();

	const int output_size = GetOutputSize();
	assert(result->count() == output_size);
	out_y.resize(output_size);

	for (int i = 0; i < output_size; ++i)
	{
		out_y[i] = result_data[i];
	}

	UnnormalizeOutput(out_y);
}

void cNeuralNet::FetchInput(Eigen::VectorXd& out_x) const
{
	const auto& input_blob = mNet->input_blobs()[0];
	const tNNData*  blob_data = input_blob->cpu_data();

	int input_size = GetInputSize();
	assert(input_blob->count() == input_size);
	out_x.resize(input_size);

	for (int i = 0; i < input_size; ++i)
	{
		out_x[i] = blob_data[i];
	}

	UnnormalizeInput(out_x);
}

void cNeuralNet::NormalizeInput(Eigen::MatrixXd& X) const
{
	if (ValidOffsetScale())
	{
		for (int i = 0; i < X.rows(); ++i)
		{
			auto curr_row = X.row(i);
			curr_row += mInputOffset;
			curr_row = curr_row.cwiseProduct(mInputScale);
		}
	}
}

void cNeuralNet::NormalizeInput(Eigen::VectorXd& x) const
{
	if (ValidOffsetScale())
	{
		assert(x.size() == mInputOffset.size());
		assert(x.size() == mInputScale.size());
		x += mInputOffset;
		x = x.cwiseProduct(mInputScale);
	}
}

void cNeuralNet::NormalizeInputDiff(Eigen::VectorXd& x_diff) const
{
	if (ValidOffsetScale())
	{
		assert(x_diff.size() == mInputScale.size());
		x_diff = x_diff.cwiseProduct(mInputScale);
	}
}

void cNeuralNet::UnnormalizeInput(Eigen::VectorXd& x) const
{
	if (ValidOffsetScale())
	{
		assert(x.size() == mInputScale.size());
		x = x.cwiseQuotient(mInputScale);
		x -= mInputOffset;
	}
}

void cNeuralNet::UnnormalizeInputDiff(Eigen::VectorXd& x_diff) const
{
	if (ValidOffsetScale())
	{
		assert(x_diff.size() == mInputScale.size());
		x_diff = x_diff.cwiseQuotient(mInputScale);
	}
}

void cNeuralNet::NormalizeOutput(Eigen::VectorXd& y) const
{
	if (ValidOffsetScale())
	{
		assert(y.size() == mOutputOffset.size());
		assert(y.size() == mOutputScale.size());
		y += mOutputOffset;
		y = y.cwiseProduct(mOutputScale);
	}
}

void cNeuralNet::UnnormalizeOutput(Eigen::VectorXd& y) const
{
	if (ValidOffsetScale())
	{
		assert(y.size() == mOutputOffset.size());
		assert(y.size() == mOutputScale.size());
		y = y.cwiseQuotient(mOutputScale);
		y -= mOutputOffset;
	}
}

void cNeuralNet::NormalizeOutputDiff(Eigen::VectorXd& y_diff) const
{
	if (ValidOffsetScale())
	{
		assert(y_diff.size() == mOutputScale.size());
		y_diff = y_diff.cwiseProduct(mOutputScale);
	}
}

void cNeuralNet::UnnormalizeOutputDiff(Eigen::VectorXd& y_diff) const
{
	if (ValidOffsetScale())
	{
		assert(y_diff.size() == mOutputScale.size());
		y_diff = y_diff.cwiseQuotient(mOutputScale);
	}
}

boost::shared_ptr<caffe::Net<cNeuralNet::tNNData>> cNeuralNet::GetTrainNet() const
{
	if (HasSolver())
	{
		return mSolver->GetNet();
	}
	return nullptr;
}

boost::shared_ptr<caffe::MemoryDataLayer<cNeuralNet::tNNData>> cNeuralNet::GetTrainDataLayer() const
{
	if (HasSolver())
	{
		auto train_net = GetTrainNet();
		const std::string& data_layer_name = GetInputLayerName();
		auto data_layer = boost::static_pointer_cast<caffe::MemoryDataLayer<tNNData>>(train_net->layer_by_name(data_layer_name));
		return data_layer;
	}
	return nullptr;
}

void cNeuralNet::LoadTrainData(const Eigen::MatrixXd& X, const Eigen::MatrixXd& Y)
{
	boost::shared_ptr<caffe::Net<tNNData>> train_net = GetTrainNet();
	auto data_layer = GetTrainDataLayer();
	int batch_size = GetBatchSize();

	int num_batches = static_cast<int>(X.rows()) / batch_size;
	assert(num_batches == 1);
	num_batches = 1;

	int num_data = num_batches * batch_size;
	int data_dim = static_cast<int>(X.cols());
	int label_dim = static_cast<int>(Y.cols());

	std::vector<tNNData> data(num_data * data_dim);
	std::vector<tNNData> labels(num_data * label_dim);

	for (int i = 0; i < num_data; ++i)
	{
		auto curr_data = X.row(i);
		auto curr_label = Y.row(i);

		for (int j = 0; j < data_dim; ++j)
		{
			double val = curr_data[j];
			if (ValidOffsetScale())
			{
				val += mInputOffset[j];
				val = val * mInputScale[j];
			}
			data[i * data_dim + j] = val;
		}

		for (int j = 0; j < label_dim; ++j)
		{
			double val = curr_label[j];
			if (ValidOffsetScale())
			{
				val += mOutputOffset[j];
				val = val * mOutputScale[j];
			}
			labels[i * label_dim + j] = val;
		}
	}

	data_layer->AddData(data, labels);
}

bool cNeuralNet::WriteData(const Eigen::MatrixXd& X, const Eigen::MatrixXd& Y, const std::string& out_file)
{
	bool succ = true;
	int num_data = static_cast<int>(X.rows());
	assert(num_data = static_cast<int>(Y.rows()));
	int x_size = static_cast<int>(X.cols());
	int y_size = static_cast<int>(Y.cols());

	std::vector<float> x_data(num_data * x_size);
	std::vector<float> y_data(num_data * y_size);

	for (int i = 0; i < num_data; ++i)
	{
		const auto& curr_x = X.row(i);
		const auto& curr_y = Y.row(i);

		for (int j = 0; j < x_size; ++j)
		{
			x_data[i * x_size + j] = static_cast<float>(curr_x(j));
		}

		for (int j = 0; j < y_size; ++j)
		{
			y_data[i * y_size + j] = static_cast<float>(curr_y(j));
		}
	}

	const int rank = 4;
	const hsize_t x_dims[rank] = { num_data, 1, 1, x_size };
	const hsize_t y_dims[rank] = { num_data, 1, 1, y_size };

	hid_t file_hid = H5Fcreate(out_file.c_str(), H5F_ACC_TRUNC, H5P_DEFAULT,
								H5P_DEFAULT);
	herr_t status = H5LTmake_dataset_float(file_hid, "data", rank, x_dims, x_data.data());
	if (status == 0)
	{
		status = H5LTmake_dataset_float(file_hid, "label", rank, y_dims, y_data.data());
	}

	if (status != 0)
	{
		succ = false;
	}

	status = H5Fclose(file_hid);
	succ &= (status == 0);
	return succ;
}

std::string cNeuralNet::GetOffsetScaleFile(const std::string& model_file) const
{
	std::string scale_file = model_file;
	scale_file = cFileUtil::RemoveExtension(scale_file);
	scale_file += "_scale.txt";
	return scale_file;
}

void cNeuralNet::WriteOffsetScale(const std::string& norm_file) const
{
	FILE* f = cFileUtil::OpenFile(norm_file, "w");

	if (f != nullptr)
	{
		std::string input_offset_json = cJsonUtil::BuildVectorJson(mInputOffset);
		std::string input_scale_json = cJsonUtil::BuildVectorJson(mInputScale);
		std::string output_offset_json = cJsonUtil::BuildVectorJson(mOutputOffset);
		std::string output_scale_json = cJsonUtil::BuildVectorJson(mOutputScale);

		fprintf(f, "{\n\"%s\": %s,\n\"%s\": %s,\n\"%s\": %s,\n\"%s\": %s\n}", 
			gInputOffsetKey.c_str(), input_offset_json.c_str(),
			gInputScaleKey.c_str(), input_scale_json.c_str(),
			gOutputOffsetKey.c_str(), output_offset_json.c_str(),
			gOutputScaleKey.c_str(), output_scale_json.c_str());

		cFileUtil::CloseFile(f);
	}
	else
	{
		printf("Failed to write offset and scale to %s\n", norm_file.c_str());
	}
}

const std::string& cNeuralNet::GetInputLayerName() const
{
	return gInputLayerName;
}

const std::string& cNeuralNet::GetOutputLayerName() const
{
	return gOutputLayerName;
}


/////////////////////////////
// Caffe Net Wrapper
/////////////////////////////

cNeuralNet::cCaffeNetWrapper::cCaffeNetWrapper(const std::string& net_file, caffe::Phase phase)
	: caffe::Net<tNNData>(net_file, phase)
{
}

cNeuralNet::cCaffeNetWrapper::~cCaffeNetWrapper()
{
}

int cNeuralNet::cCaffeNetWrapper::GetLayerIdx(const std::string& layer_name) const
{
	int idx = layer_names_index_.find(layer_name)->second;
	return idx;
}

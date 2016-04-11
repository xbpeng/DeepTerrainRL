#include "BaseControllerMACE.h"

#define ENABLE_BOLTZMANN_EXP
//#define ENABLE_LAYER_NOISE
//#define ENABLE_ACTOR_BIAS_NOISE
//#define ENABLE_COVAR_ACTION_EXP

cBaseControllerMACE::cBaseControllerMACE() : cTerrainRLCharController()
{
	mExpBaseActionRate = 0.2;
	mExpNoise = 0.2;
	mNumActionFrags = 0;
	mExpCritic = false;
	mExpActor = false;
	mExpLayer = "";
}

cBaseControllerMACE::~cBaseControllerMACE()
{
}

void cBaseControllerMACE::Reset()
{
	mExpCritic = false;
	mExpActor = false;
}

int cBaseControllerMACE::GetPoliActionSize() const
{
	return 1 + GetActionFragSize();
}

int cBaseControllerMACE::GetNetOutputSize() const
{
	return mNumActionFrags + mNumActionFrags * GetActionFragSize();
}

int cBaseControllerMACE::GetNumActionFrags() const
{
	return mNumActionFrags;
}

int cBaseControllerMACE::GetActionFragSize() const
{
	return GetNumOptParams();
}

bool cBaseControllerMACE::IsExpCritic() const
{
	return mExpCritic;
}

bool cBaseControllerMACE::IsExpActor() const
{
	return mExpActor;
}

void cBaseControllerMACE::RecordPoliAction(Eigen::VectorXd& out_action) const
{
	out_action = Eigen::VectorXd::Zero(GetPoliActionSize());

	int a = mCurrAction.mID;
	cMACETrainer::SetActionFragIdx(a, out_action);

	Eigen::VectorXd frag;
	BuildOptParams(frag);
	cMACETrainer::SetActionFrag(frag, out_action);
}

void cBaseControllerMACE::SetExpLayer(const std::string& layer_name)
{
	mExpLayer = layer_name;
}

void cBaseControllerMACE::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	Eigen::VectorXd action_frag_offset;
	Eigen::VectorXd action_frag_scale;
	BuildActionFragOutputOffsetScale(action_frag_offset, action_frag_scale);

	int output_size = GetNetOutputSize();
	int num_frags = GetNumActionFrags();
	assert(output_size == num_frags + num_frags * action_frag_offset.size());

	out_offset = Eigen::VectorXd::Zero(output_size);
	out_scale = Eigen::VectorXd::Ones(output_size);

	for (int f = 0; f < num_frags; ++f)
	{
		Eigen::VectorXd curr_frag_offset;
		BuildActorBias(f, curr_frag_offset);
		curr_frag_offset = -curr_frag_offset;

#if defined(DISABLE_INIT_ACTOR_BIAS)
		curr_frag_offset = action_frag_offset;
#endif

#if defined(ENABLE_ACTOR_BIAS_NOISE)
		const double noise_scale = 0.5;
		for (int i = 0; i < static_cast<int>(curr_frag_offset.size()); ++i)
		{
			double curr_scale = 1.0 / action_frag_scale[i];
			double rand_noise = cMathUtil::RandDoubleNorm(0, noise_scale);
			curr_frag_offset[i] += curr_scale * rand_noise;
		}
#endif

		SetVal(-0.5, f, out_offset);
		SetVal(2, f, out_scale);
		SetFrag(curr_frag_offset, f, out_offset);
		SetFrag(action_frag_scale, f, out_scale);
	}
}

void cBaseControllerMACE::LoadNetIntern(const std::string& net_file)
{
	cTerrainRLCharController::LoadNetIntern(net_file);
	UpdateFragParams();
}

void cBaseControllerMACE::UpdateFragParams()
{
	int num_outputs = mNet.GetOutputSize();
	mNumActionFrags = cMACETrainer::CalcNumFrags(num_outputs, GetActionFragSize());

#if defined(ENABLE_BOLTZMANN_EXP)
	mBoltzmannBuffer.resize(mNumActionFrags);
#endif
}

void cBaseControllerMACE::BuildActionFragOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int action_frag_size = GetActionFragSize();

	int num_actions = GetNumActions();
	if (num_actions > 0)
	{
		int default_action_id = GetDefaultAction();
		if (default_action_id == gInvalidIdx)
		{
			default_action_id = 0;
		}

		out_offset = Eigen::VectorXd::Zero(action_frag_size);
		out_scale = Eigen::VectorXd::Ones(action_frag_size);

		BuildActionOptParams(default_action_id, out_offset);
		out_offset *= -1;

		if (num_actions > 1)
		{
			out_scale.setZero();
			Eigen::VectorXd param_buffer;
			for (int a = 0; a < num_actions; ++a)
			{
				if (a != default_action_id)
				{
					BuildActionOptParams(a, param_buffer);
					param_buffer += out_offset;
					param_buffer = param_buffer.cwiseAbs();
					out_scale = out_scale.cwiseMax(param_buffer);
				}
			}

			out_scale = out_scale.cwiseInverse();
		}
	}
}

void cBaseControllerMACE::UpdateAction()
{
	mExpActor = false;
	mExpCritic = false;
}

void cBaseControllerMACE::DecideAction(tAction& out_action)
{
#if defined(ENABLE_BOLTZMANN_EXP)
	DecideActionBoltzmann(out_action);
#else
	cDogControllerCacla::DecideAction(out_action);
#endif
}

void cBaseControllerMACE::ExploitPolicy(tAction& out_action)
{
	Eigen::VectorXd y;
	mNet.Eval(mPoliState, y);

	int a = GetMaxFragIdx(y);
	double val = GetVal(y, a);
	BuildActorAction(y, a, out_action);

#if defined(ENABLE_DEBUG_VISUALIZATION)
	mPoliValLog.Add(val);
	mVisNNOutput = y;
#endif // ENABLE_DEBUG_VISUALIZATION

#if defined (ENABLE_DEBUG_PRINT)
	PrintNetOutput(y, a);
#endif
}

void cBaseControllerMACE::ExploreAction(tAction& out_action)
{
	const double critic_exp_val = 0.6;
	const double actor_exp_val = 0.8;

	double base_rand = cMathUtil::RandDouble();
	if (base_rand < mExpBaseActionRate)
	{
		BuildRandBaseAction(out_action);
		mExpActor = true;
		mExpCritic = true;
	}
	else
	{
		double rand = cMathUtil::RandDouble();
		if (rand < critic_exp_val)
		{
			GetRandActorAction(out_action);
			mExpCritic = true;
			mExpActor = false;
		}
		else if (rand < actor_exp_val)
		{
			ExploitPolicy(out_action);
			ApplyExpNoise(out_action);
			mExpCritic = false;
			mExpActor = true;
		}
		else
		{
			GetRandActorAction(out_action);
			ApplyExpNoise(out_action);
			mExpCritic = true;
			mExpActor = true;
		}

#if defined (ENABLE_DEBUG_PRINT)
		if (mExpActor)
		{
			printf("Actor ");
		}
		if (mExpCritic)
		{
			printf("Critic ");
		}
		printf("Exploration\n");
#endif
	}
}

void cBaseControllerMACE::DecideActionBoltzmann(tAction& out_action)
{
	mIsOffPolicy = false;
	double base_rand = cMathUtil::RandDouble();
	if (mEnableExp && base_rand < mExpBaseActionRate)
	{
		BuildRandBaseAction(out_action);
		mIsOffPolicy = true;
		mExpActor = true;
		mExpCritic = true;
	}
	else
	{
		Eigen::VectorXd y;
		mNet.Eval(mPoliState, y);

		int a_max = GetMaxFragIdx(y);
		int a = a_max;

		if (mEnableExp)
		{
			a = BoltzmannSelectActor(y, mBoltzmannBuffer);
		}

		double val = GetVal(y, a);
		BuildActorAction(y, a, out_action);

		if (mEnableExp)
		{
			double rand_noise = cMathUtil::RandDouble();
			if (rand_noise < mExpRate)
			{
				ApplyExpNoise(out_action);
				mExpActor = true;
			}

			mExpCritic = (a != a_max);
			mIsOffPolicy = mExpActor || mExpCritic;
		}
		
		
#if defined(ENABLE_DEBUG_VISUALIZATION)
		mPoliValLog.Add(val);
		mVisNNOutput = y;
#endif // ENABLE_DEBUG_VISUALIZATION

#if defined (ENABLE_DEBUG_PRINT)
		if (mExpActor || mExpCritic)
		{
			printf("\n");
			if (mExpActor)
			{
				printf("Actor ");
			}
			if (mExpCritic)
			{
				printf("Critic ");
			}
			printf("Exploration\n");
		}

		PrintNetOutput(y, a);
#endif
	}
}

void cBaseControllerMACE::GetRandActorAction(tAction& out_action)
{
	Eigen::VectorXd y;
	mNet.Eval(mPoliState, y);

	int max_a = GetMaxFragIdx(y);
	int a = cMathUtil::RandIntExclude(0, GetNumActionFrags(), max_a);
	double val = GetVal(y, a);
	BuildActorAction(y, a, out_action);

#if defined(ENABLE_DEBUG_VISUALIZATION)
	mPoliValLog.Add(val);
#endif // ENABLE_DEBUG_VISUALIZATION

#if defined (ENABLE_DEBUG_PRINT)
	PrintNetOutput(y, a);
#endif
}

void cBaseControllerMACE::BuildActorAction(const Eigen::VectorXd& params, int a_id, tAction& out_action) const
{
	Eigen::VectorXd action_frag;
	GetFrag(params, a_id, action_frag);
	assert(action_frag.size() == GetNumOptParams());

	out_action.mID = a_id;
	out_action.mParams = mCurrAction.mParams;
	SetOptParams(action_frag, out_action.mParams);
}

int cBaseControllerMACE::BoltzmannSelectActor(const Eigen::VectorXd& params, Eigen::VectorXd& val_buffer) const
{
	int a_max = GetMaxFragIdx(params);
	int a = a_max;

	if (mExpTemp != 0)
	{
		int num_actors = GetNumActionFrags();
		double max_val = GetVal(params, a_max);

		double sum = 0;
		for (int i = 0; i < num_actors; ++i)
		{
			double curr_val = GetVal(params, i);
			curr_val = std::exp((curr_val - max_val) / mExpTemp);

			val_buffer[i] = curr_val;
			sum += curr_val;
		}

		double rand = cMathUtil::RandDouble(0, sum);
		for (int i = 0; i < num_actors; ++i)
		{
			double curr_val = val_buffer[i];
			rand -= curr_val;

			if (rand <= 0)
			{
				a = i;
				break;
			}
		}

#if defined (ENABLE_DEBUG_PRINT)
		printf("Boltzmann:\t");
		for (int i = 0; i < num_actors; ++i)
		{
			double curr_val = val_buffer[i];
			curr_val /= sum;
			printf("%.3f\t", curr_val);
		}
		printf("\n");
#endif
	}

	return a;
}

void cBaseControllerMACE::ApplyExpNoise(tAction& out_action)
{
#if defined(ENABLE_LAYER_NOISE)
	bool heads = cMathUtil::FlipCoin();
	if (heads || !ValidExpLayer())
	{
		ApplyExpNoiseAction(out_action);
	}
	else
	{
		ApplyExpNoiseState(out_action);
	}
#else
	ApplyExpNoiseAction(out_action);
#endif
}

void cBaseControllerMACE::ApplyExpNoiseState(tAction& out_action)
{
	if (ValidExpLayer())
	{
		const double noise_mean = 0;
		const double noise_stdev = 0.2;

		Eigen::VectorXd y;
		mNet.ForwardInjectNoisePrefilled(noise_mean, noise_stdev, mExpLayer, y);
		BuildActorAction(y, out_action.mID, out_action);
	}
	else
	{
		assert(false); // invalid layer
	}
}

bool cBaseControllerMACE::ValidExpLayer() const
{
	return mExpLayer != "" && mNet.HasLayer(mExpLayer);
}

void cBaseControllerMACE::ApplyExpNoiseAction(tAction& out_action)
{
#if defined(ENABLE_COVAR_ACTION_EXP)
	const int num_samples = 16;
	const double noise_mean = 0;
	const double noise_stdev = 0.2;
	const double reg = 0.01;

	int action_size = static_cast<int>(out_action.mParams.size() - 1);
	Eigen::MatrixXd samples(num_samples, action_size);

	Eigen::VectorXd layer_state;
	mNet.GetLayerState(mExpLayer, layer_state);

	Eigen::VectorXd action_mean;
	GetOptParams(out_action.mParams, action_mean);
	samples.row(0) = action_mean;

	for (int i = 1; i < num_samples; ++i)
	{
		Eigen::VectorXd y;
		mNet.SetLayerState(layer_state, mExpLayer);
		mNet.ForwardInjectNoisePrefilled(noise_mean, noise_stdev, mExpLayer, y);

		Eigen::VectorXd curr_action;
		GetFrag(y, out_action.mID, curr_action);

		samples.row(i) = curr_action;
		action_mean += curr_action;
	}

	action_mean /= num_samples;

	Eigen::VectorXd noise_scale;
	FetchExpNoiseScale(noise_scale);
	assert(noise_scale.size() == action_size);

	double cov_scale = 10.0 / num_samples;
	Eigen::MatrixXd cov_mat = reg * Eigen::MatrixXd::Identity(action_size, action_size);
	for (int i = 0; i < num_samples; ++i)
	{
		Eigen::VectorXd curr_action = samples.row(i);
		curr_action -= action_mean;
		curr_action = curr_action.cwiseQuotient(noise_scale);
		cov_mat += (cov_scale * curr_action) * curr_action.transpose();
	}

	Eigen::VectorXd noise(action_size);
	double exp_noise_stdev = 5;
	for (int i = 0; i < action_size; ++i)
	{
		noise[i] = cMathUtil::RandDoubleNorm(0, exp_noise_stdev);
	}

	SetOptParams(action_mean + (cov_mat * noise).cwiseProduct(noise_scale), out_action.mParams);
#else
	int num_params = GetNumParams();
	int num_opt_params = GetNumOptParams();
	Eigen::VectorXd noise_scale;
	FetchExpNoiseScale(noise_scale);

	assert(noise_scale.size() == num_opt_params);

	// for debugging
	Eigen::VectorXd exp_noise = Eigen::VectorXd::Zero(num_opt_params);

	int opt_idx = 0;
	for (int i = 0; i < num_params; ++i)
	{
		if (IsOptParam(i))
		{
			double noise = cMathUtil::RandDoubleNorm(0, mExpNoise);
			double scale = noise_scale[opt_idx];
			noise *= scale;

			out_action.mParams[i] += noise;
			exp_noise[opt_idx] = noise;
			++opt_idx;
		}
	}
#endif
}

void cBaseControllerMACE::ProcessCommand(tAction& out_action)
{
	mExpActor = true;
	mExpCritic = true;
}

int cBaseControllerMACE::GetMaxFragIdx(const Eigen::VectorXd& params) const
{
	return cMACETrainer::GetMaxFragIdx(params, GetNumActionFrags());
}

double cBaseControllerMACE::GetMaxFragVal(const Eigen::VectorXd& params) const
{
	return cMACETrainer::GetMaxFragVal(params, GetNumActionFrags());
}

void cBaseControllerMACE::GetFrag(const Eigen::VectorXd& params, int a_idx, Eigen::VectorXd& out_action) const
{
	cMACETrainer::GetFrag(params, GetNumActionFrags(), GetActionFragSize(), a_idx, out_action);
}

void cBaseControllerMACE::SetFrag(const Eigen::VectorXd& frag, int a_idx, Eigen::VectorXd& out_params) const
{
	cMACETrainer::SetFrag(frag, a_idx, GetNumActionFrags(), GetActionFragSize(), out_params);
}

double cBaseControllerMACE::GetVal(const Eigen::VectorXd& params, int a_idx) const
{
	return cMACETrainer::GetVal(params, a_idx);
}

void cBaseControllerMACE::SetVal(double val, int a_idx, Eigen::VectorXd& out_params) const
{
	cMACETrainer::SetVal(val, a_idx, out_params);
}

void cBaseControllerMACE::FetchExpNoiseScale(Eigen::VectorXd& out_noise) const
{
	const Eigen::VectorXd& nn_output_scale = mNet.GetOutputScale();
	GetFrag(nn_output_scale, 0, out_noise);
	out_noise = out_noise.cwiseInverse();
}

void cBaseControllerMACE::BuildActorBias(int a_id, Eigen::VectorXd& out_bias) const
{
	int action_size = GetActionFragSize();
	out_bias = Eigen::VectorXd::Zero(action_size);
}

#if defined (ENABLE_DEBUG_PRINT)
void cBaseControllerMACE::PrintNetOutput(const Eigen::VectorXd& y, int a_id) const
{
	double val = GetVal(y, a_id);
	Eigen::VectorXd action_params;
	GetFrag(y, a_id, action_params);

	printf("Action: %i\n", a_id);
	printf("Val: (%.3f)\t", val);
	for (int f = 0; f < GetNumActionFrags(); ++f)
	{
		double curr_val = GetVal(y, f);
		printf("%.3f\t", curr_val);
	}
	printf("\n");
}
#endif


#if defined(ENABLE_DEBUG_VISUALIZATION)
void cBaseControllerMACE::GetVisActionFeatures(Eigen::VectorXd& out_features) const
{
	BuildOptParams(out_features);
	if (HasNet())
	{
		Eigen::VectorXd offset;
		Eigen::VectorXd scale;
		// use the same offset and scale for all actors for easier visual comparisons
		GetFrag(mNet.GetOutputOffset(), 0, offset);
		GetFrag(mNet.GetOutputScale(), 0, scale);
		out_features = scale.cwiseProduct(out_features + offset);
	}
}

void cBaseControllerMACE::GetVisActionValues(Eigen::VectorXd& out_vals) const
{
	if (mVisNNOutput.size() > 0)
	{
		out_vals = mVisNNOutput.segment(0, GetNumActionFrags());
	}
	else
	{
		out_vals.resize(0);
	}
}
#endif
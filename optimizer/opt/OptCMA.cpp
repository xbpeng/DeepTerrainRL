#include "OptCMA.h"
#include <limits>
#include <ctime>
#include <thread>

#include "util/MathUtil.h"

const float gDefaultObjVal = std::numeric_limits<float>::infinity();

cOptCMA::tCMALog::tCMALog()
{
	mNumGens = 0;
	mFinalStepSize = 0.f;
	mTime = 0.f;
}

cOptCMA::cOptCMA() : cOptimizer()
{
	Parameters<double> cma_params;
	// dummy init
	cma_params.init(1, nullptr, nullptr);
	mCMA.init(cma_params);
}

cOptCMA::~cOptCMA(void)
{
}

const cOptFunc::tPoint& cOptCMA::Optimize(const std::string& filename_noext, bool output_logs)
{
	mLog = tCMALog();
	clock_t beg_time = clock();
	
	// eval the original state
	InitPt(mBestPt);
	double init_val = mOptFunc->EvalPt(mBestPt);
	mBestObjVal = init_val;

	if (mEnableIntOutput)
	{
		cOptimizer::OutputPt(mBestPt, mBestObjVal, filename_noext);
	}
	
	double* const* pop = NULL;
	int i = 0;
	for (i = 0; i < mParams.mMaxIter; ++i)
	{		
		InitCMA();

		int lambda = static_cast<int>(mCMA.get(CMAES<double>::Lambda));
		std::vector<double> func_vals;
		func_vals.resize(lambda);

		do
		{
			pop = mCMA.samplePopulation();
			EvalPop(pop, lambda, func_vals);

			int curr_gen = GetCurrGen();
			mCMA.updateDistribution(func_vals.data());

			int num_evals = GetEvalCount();
			double sigma = GetCurrSigma();
			double gen_best_val = std::numeric_limits<double>::infinity();
			int best_idx = -1;
			for (int i = 0; i < lambda; ++i)
			{
				if (func_vals[i] < gen_best_val)
				{
					gen_best_val = func_vals[i];
					best_idx = i;
				}
			}
			
			// output solutions that are within some range of the best
			cOptFunc::tPoint curr_pt = cOptFunc::BuildPt(pop[best_idx], mDimensions);
			if (gen_best_val < mBestObjVal)
			{
				mBestObjVal = gen_best_val;
				mBestPt = curr_pt;
			}

			if (mEnableIntOutput)
			{
				OutputPt(curr_pt, gen_best_val, filename_noext);
			}

			std::printf("Gen: %u, Eval: %i, Sigma: %.6f, CurrBest:, %.6f, Best: %.6f\n",
				curr_gen, num_evals, sigma, gen_best_val, mBestObjVal);

		} while (!CheckTermination());

		int curr_gen = GetCurrGen();
		mLog.mNumGens += curr_gen;
	}

	clock_t end_time = clock();
	double hours = static_cast<double>((end_time - beg_time) / (CLOCKS_PER_SEC * 3600.0));
	mLog.mTime = hours;	
	mLog.mFinalStepSize = GetCurrSigma();
	mLog.mInitVal = init_val;
	mLog.mObjVal = mBestObjVal;

	return mBestPt;
}

void cOptCMA::InitCMA()
{
	cOptFunc::tPoint pt0;
	mOptFunc->InitPt(pt0);

	Parameters<double> cma_params;
	cOptFunc::tPoint scale = mScale * mParams.mSigma;
	cma_params.init(mDimensions, pt0.data(), scale.data());

	if (mParams.mPopSize > 0)
	{
		cma_params.lambda = mParams.mPopSize;
		cma_params.mu = cma_params.lambda / 2;
	}

	mCMA.init(cma_params);
}


void cOptCMA::InitPt(cOptFunc::tPoint& state)
{
	mOptFunc->InitPt(state);
}

void cOptCMA::SetParams(const tCMAParams& params)
{
	mParams = params;
}

const cOptCMA::tCMALog& cOptCMA::GetLog() const
{
	return mLog;
}


bool cOptCMA::CheckTermination()
{
	int curr_gen = GetCurrGen();
	double sigma = GetCurrSigma();
	return curr_gen >= mParams.mMaxGen || mCMA.testForTermination()
		|| sigma < mParams.mStepTol;
}

int cOptCMA::GetCurrGen() const
{
	int curr_gen = static_cast<int>(mCMA.get(CMAES<double>::Generation));
	return curr_gen;
}

double cOptCMA::GetCurrSigma() const
{
	double sigma = mCMA.get(CMAES<double>::Sigma);
	double sig_scale = 1 / std::sqrt(mScale.squaredNorm() / mScale.size());
	sigma *= sig_scale;
	return sigma;
}

int cOptCMA::GetEvalCount() const
{
	int evals = static_cast<int>(mCMA.get(CMAES<double>::Eval));
	return evals;
}

cOptFunc::tPoint cOptCMA::GetCurrBestPt() const
{
	const double* best_x = mCMA.getPtr(CMAES<double>::XBestEver);
	cOptFunc::tPoint pt = cOptFunc::BuildPt(best_x, mDimensions);
	return pt;
}

void cOptCMA::EvalPop(double* const* pop, int pop_size, std::vector<double>& out_func_vals)
{
	bool thread_safe = mOptFunc->IsThreadSafe();

	if (thread_safe)
	{
		std::vector<std::thread> threads;
		threads.resize(pop_size);
		for (int i = 0; i < pop_size; ++i)
		{
			cOptFunc::tPoint curr_pt = cOptFunc::BuildPt(pop[i], mDimensions);
			std::thread& curr_thread = threads[i];
			curr_thread = std::thread(&cOptCMA::EvalPt, this, curr_pt, &(out_func_vals[i]));
		}

		for (int i = 0; i < pop_size; ++i)
		{
			threads[i].join();
		}
	}
	else
	{
		for (int i = 0; i < pop_size; ++i)
		{
			cOptFunc::tPoint curr_pt = cOptFunc::BuildPt(pop[i], mDimensions);
			EvalPt(curr_pt, &(out_func_vals[i]));
		}
	}
}

void cOptCMA::EvalPt(const cOptFunc::tPoint& pt, double* out_val)
{
	*out_val = mOptFunc->EvalPt(pt);
}

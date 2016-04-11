#include "Optimizer.h"
#include <limits>
#include <cstdlib>
#include <stdio.h>
#include <assert.h>

#include "util/FileUtil.h"
#include "util/MathUtil.h"

const double gDefaultObjVal = std::numeric_limits<double>::infinity();

cOptimizer::cOptimizer(void)
{
	mOptFunc = NULL;
	mBestObjVal = gDefaultObjVal;
	mEnableIntOutput = true;
}


void cOptimizer::Init(std::shared_ptr<cOptFunc>& opt_func)
{
	mOptFunc = opt_func;
	mDimensions = mOptFunc->GetDimensions();
	mPt.resize(mDimensions);
	mBestPt.resize(mDimensions);

	mOptFunc->InitScale(mScale);
	assert(mScale.size() == mDimensions);
}

void cOptimizer::OutputIntermediateResult(bool enable)
{
	mEnableIntOutput = enable;
}


// IO stuff
void cOptimizer::OutputPt(const cOptFunc::tPoint& pt, double obj_val, const std::string& filename_noext) const
{
	char suffix[64];
	sprintf(suffix, "_%.10f.txt", obj_val);
	std::string filename = filename_noext + suffix;
	FILE* file_ptr = cFileUtil:: OpenFile(filename.c_str(), "w");

	mOptFunc->OutputPt(file_ptr, pt);
		
	cFileUtil::CloseFile(file_ptr);
}

void cOptimizer::OutputLog(const std::vector<double>& fit_vals, 
						   double final_val, const std::string& filename_noext) const
{
	char suffix[64];
	sprintf(suffix, "_%.4f_log.txt", final_val);
	std::string filename = filename_noext + suffix;
	FILE* file_ptr = cFileUtil::OpenFile(filename.c_str(), "w");

	for (size_t i = 0; i < fit_vals.size(); ++i)
	{
		fprintf(file_ptr, "%.8f\n", fit_vals[i]);
	}
		
	cFileUtil::CloseFile(file_ptr);
}

cOptimizer::~cOptimizer(void)
{
}

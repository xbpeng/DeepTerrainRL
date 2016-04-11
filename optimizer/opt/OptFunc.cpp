#include "OptFunc.h"
#include <assert.h>
#include "Optimizer.h"
#include "util//FileUtil.h"

cOptFunc::cOptFunc(void)
{
}

cOptFunc::tPoint cOptFunc::BuildPt(const double* x, int dimensions)
{
	cOptFunc::tPoint pt(dimensions);
	for (int i = 0; i < dimensions; ++i)
	{
		pt[i] = x[i];
	}
	return pt;
}

cOptFunc::~cOptFunc(void)
{
}

bool cOptFunc::IsThreadSafe() const
{
	return false;
}

void cOptFunc::OutputPt(const std::string& filename, const tPoint& pt) const
{
	FILE* f = cFileUtil::OpenFile(filename.c_str(), "w");

	OutputPt(f, pt);

	cFileUtil::CloseFile(f);
}

void cOptFunc::OutputPt(FILE* f, const tPoint& pt) const
{
	// does nothing
}


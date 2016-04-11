#pragma once

#include <vector>
#include "util/MathUtil.h"

class cOptFunc
{
public:
	typedef Eigen::VectorXd tPoint;

	static tPoint BuildPt(const double* x, int dimensions);

	cOptFunc(void);
	virtual ~cOptFunc(void);

	virtual int GetDimensions() const = 0;
	virtual void InitPt(tPoint& pt) const = 0;
	virtual double EvalPt(const tPoint& pt) = 0;
	virtual void InitScale(tPoint& scale) const = 0;

	virtual bool IsThreadSafe() const;

	virtual void OutputPt(const std::string& filename, const tPoint& pt) const;
	virtual void OutputPt(FILE* f, const tPoint& pt) const;

private:
};
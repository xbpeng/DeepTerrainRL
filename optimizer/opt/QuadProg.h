#pragma once

#include "util/MathUtil.h"

// tries to MINIMIZE a function as represented by a cOptFunc
class cQuadProg
{
public:
	struct tProb
	{
		tProb();
		tProb(int n);

		int GetDim() const;
		int GetNumCons() const;

		Eigen::MatrixXd mC; // quadratic objective
		Eigen::VectorXd md; // linear objective
		Eigen::MatrixXd mA; // constraint matrix
		Eigen::VectorXd mb; // constraint vector
		Eigen::VectorXd mLower; // variable lower bounds
		Eigen::VectorXd mUpper; // varaible upper bounds
		Eigen::VectorXd mInitX; // init value of optimization variables
		int mNumEquality; //  equality constraints must be specified before equality constraints
		double mEps; // tolerance
		int mMode; // set to 0 if c is actually the cholesky decomposition of the quadratic objective, 1 otherwise
	};

	struct tSoln
	{
		Eigen::VectorXd mX;
	};

	static tProb BuildProb(int n);
	static int Solve(const tProb& prob, tSoln& out_soln);
	static bool CheckProb(const tProb& prob);

protected:
	static bool CheckFail(long int ifail);
};
#include "QuadProg.h"
/*
#include "sfe.c"
#include "pow_di.c"
#include "c_sqrt.c"
#include "wsfe.c"
#include "fmt.c"
*/ 
// extern "C" {
// #include "opt/src/QL.c"
// }

extern "C" {
int MAIN__() {
	return 0;
}
#include "f2c.h"
// Forward definition.
int ql_(integer *m, integer *me, integer *mmax, integer *n, 
	integer *nmax, integer *mnn, doublereal *c__, doublereal *d__, 
	doublereal *a, doublereal *b, doublereal *xl, doublereal *xu, 
	doublereal *x, doublereal *u, doublereal *eps, integer *mode, integer 
	*iout, integer *ifail, integer *iprint, doublereal *war, integer *
	lwar, integer *iwar, integer *liwar);
}

cQuadProg::tProb::tProb()
	: tProb(0)
{
	
}

cQuadProg::tProb::tProb(int n)
{
	mNumEquality = 0;
	mEps = 1.0e-8;
	mMode = 1;

	mC = Eigen::MatrixXd::Zero(n, n);
	md = Eigen::VectorXd::Zero(n);
	mA.resize(0, 0);
	mb.resize(0);
	mLower = Eigen::VectorXd::Ones(n) * -INFINITY;
	mUpper = Eigen::VectorXd::Ones(n) * INFINITY;
	mInitX = Eigen::VectorXd::Zero(n);
}

int cQuadProg::tProb::GetDim() const
{
	return md.size();
}

int cQuadProg::tProb::GetNumCons() const
{
	return mA.rows();
}

// solves
// arg_min(x)	1/2 * x^T * C * x + d^t * x;
// st			a_j^T + b_j = 0 , j = 0,..., mNumEqality - 1
//				a_j^T + b_j >= 0, j = mNumEquality,..., m
//				x_l <= x <= x_u

int cQuadProg::Solve(const tProb& prob, tSoln& out_soln)
{
	assert(CheckProb(prob));

	int m = prob.GetNumCons(); // number of constraints
	int me = prob.mNumEquality; // number of equality constraints
	int mmax = m; // number of linear constraints
	int n = prob.GetDim(); // number of optimization variables
	int nmax = n; // number of bounded variables
	int mnn = m + n + n;

	out_soln.mX = prob.mInitX;
	Eigen::VectorXd u_buffer(mnn);

	// arg const cast....but c subroutine should still be safe....i think....
	double* c = const_cast<double*>(prob.mC.data());
	double* d = const_cast<double*>(prob.md.data());
	double* a = const_cast<double*>(prob.mA.data());
	double* b = const_cast<double*>(prob.mb.data());
	double* xl = const_cast<double*>(prob.mLower.data());
	double* xu = const_cast<double*>(prob.mUpper.data());
	double* x = out_soln.mX.data();
	double* u = u_buffer.data();
	double eps = prob.mEps;
	int mode = prob.mMode;
	int iout = 1; // where to rite output messages
	int ifail = 0; // output failure code.
	int iprint = 0; // output verbosity level (0 for nothing).

	int lwar = 3 * n*n + 10 * n + mmax + m + 1;
	Eigen::VectorXd war_buffer(lwar);
	double *war = war_buffer.data();

	int liwar = n;
	std::vector<int> iwar_buffer(liwar);
	int *iwar = iwar_buffer.data();

	// solve QP // an integer maps to an int
	ql_(&m, &me, &mmax, &n, &nmax, &mnn, c, d, a, b, xl, xu, x, u, &eps, &mode, &iout,
		&ifail, &iprint, war, &lwar, iwar, &liwar);

	bool fail = CheckFail(ifail);
	assert(!fail);
	return ifail;
}

bool cQuadProg::CheckProb(const tProb& prob)
{
	int n = prob.GetDim();
	int m = prob.GetNumCons();

	bool valid = true;
	if (!(prob.mC.rows() == n && prob.mC.rows() == n))
	{
		printf("QP Error: mC: %i x %i, expected %i x %i\n", prob.mC.rows(), prob.mC.cols(), n, n);
		valid = false;
	}

	if (!(prob.md.size() == n))
	{
		printf("QP Error: md: %i x 1, expected %i x 1\n", prob.mC.size(), n);
		valid = false;
	}

	if (!(prob.mA.cols() == n))
	{
		printf("QP Error: mA has %i columnss, expeted %i", prob.mA.cols(), n, n);
		valid = false;
	}

	if (!(prob.mA.rows() == prob.mb.rows()))
	{
		printf("QP Error: mA has %i rows, mb has size %i", prob.mA.rows(), prob.mb.size());
		valid = false;
	}

	if (!(prob.mLower.size() == n && prob.mUpper.size() == n))
	{
		printf("QP Error: mLower size: %i, mUpper size: %i, expected: %i", prob.mLower.size(), prob.mUpper.size(), n);
		valid = false;
	}

	if (prob.mNumEquality > m)
	{
		printf("QP Error: %i equality constraints, but only %i constraints in total", prob.mNumEquality, m);
		valid = false;
	}
	return valid;
}

bool cQuadProg::CheckFail(long int ifail)
{
	bool fail = true;
	switch (ifail)
	{
	case 0:
		// optimality ocnditions satisfied
		fail = false;
		break;
	case 1:
		printf("Termination after too many iterations\n");
		break;
	case 2:
		printf("Termination accuracy insufficient\n");
		break;
	case 3:
		printf("Inconsistency, division by zero\n");
		break;
	case 4:
		printf("Numerical instabilities\n");
		break;
	case 5:
		printf("LWAR, LIWAR, MNN, or EPS incorrect\n");
		break;
	default:
		printf("Inconsistent constraints\n");
		break;
	}

	return fail;
}

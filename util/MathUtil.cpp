#include "MathUtil.h"
#include <time.h>

cRand cMathUtil::gRand = cRand();

int cMathUtil::Clamp(int val, int min, int max)
{
	return std::max(min, std::min(val, max));
}

double cMathUtil::Clamp(double val, double min, double max)
{
	return std::max(min, std::min(val, max));
}

double cMathUtil::Saturate(double val)
{
	return Clamp(val, 0.0, 1.0);
}

double cMathUtil::RandDouble()
{
	return RandDouble(0, 1);
}

double cMathUtil::RandDouble(double min, double max)
{
	return gRand.RandDouble(min, max);
}

double cMathUtil::RandDoubleNorm(double mean, double stdev)
{
	return gRand.RandDoubleNorm(mean, stdev);
}

int cMathUtil::RandInt(int min, int max)
{
	return gRand.RandInt(min, max);
}

int cMathUtil::RandIntExclude(int min, int max, int exc)
{
	return gRand.RandIntExclude(min, max, exc);
}

void cMathUtil::SeedRand(unsigned long int seed)
{
	gRand.Seed(seed);
}

int cMathUtil::RandSign()
{
	return gRand.RandSign();
}

double cMathUtil::SmoothStep(double t)
{
	double val = t * t * t * (t * (t * 6 - 15) + 10);
	return val;
}

bool cMathUtil::FlipCoin(double p)
{
	return gRand.FlipCoin(p);
}

tMatrix cMathUtil::TranslateMat(const tVector& trans)
{
	tMatrix mat = tMatrix::Identity();
	mat(0, 3) = trans[0];
	mat(1, 3) = trans[1];
	mat(2, 3) = trans[2];
	return mat;
}

tMatrix cMathUtil::ScaleMat(double scale)
{
	return ScaleMat(tVector::Ones() * scale);
}

tMatrix cMathUtil::ScaleMat(const tVector& scale)
{
	tMatrix mat = tMatrix::Identity();
	mat(0, 0) = scale[0];
	mat(1, 1) = scale[1];
	mat(2, 2) = scale[2];
	return mat;
}

tMatrix cMathUtil::RotateMat(const tVector& axis, double theta)
{
	assert(std::abs(axis.squaredNorm() - 1) < 0.01);
	
	double c = std::cos(theta);
	double s = std::sin(theta);
	double x = axis[0];
	double y = axis[1];
	double z = axis[2];

	tMatrix mat;
	mat <<	c + x * x * (1 - c),		x * y * (1 - c) - z * s,	x * z * (1 - c) + y * s,	0,
			y * x * (1 - c) + z * s,	c + y * y * (1 - c),		y * z * (1 - c) - x * s,	0,
			z * x * (1 - c) - y * s,	z * y * (1 - c) + x * s,	c + z * z * (1 - c),		0,
			0,							0,							0,							1;

	return mat;
}

tMatrix cMathUtil::CrossMat(const tVector& a)
{
	tMatrix m;
	m << 0,		-a[2],	a[1],	0,
		 a[2],	0,		-a[0],	0,
		 -a[1],	a[0],	0,		0,
		 0,		0,		0,		1;
	return m;
}

tMatrix cMathUtil::InvRigidMat(const tMatrix& mat)
{
	tMatrix inv_mat = tMatrix::Zero();
	inv_mat.block(0, 0, 3, 3) = mat.block(0, 0, 3, 3).transpose();
	inv_mat.col(3) = -inv_mat * mat.col(3);
	inv_mat(3, 3) = 1;
	return inv_mat;
}

void cMathUtil::RotMatToAxisAngle(const tMatrix& mat, tVector& out_axis, double& out_theta)
{
	double c = (mat(0, 0) + mat(1, 1) + mat(2, 2) - 1) * 0.5;
	c = cMathUtil::Clamp(c, -1.0, 1.0);

	out_theta = std::acos(c);
	if (std::abs(out_theta) < 0.00001)
	{
		out_axis = tVector(0, 0, 1, 0);
	}
	else
	{
		double m21 = mat(2, 1) - mat(1, 2);
		double m02 = mat(0, 2) - mat(2, 0);
		double m10 = mat(1, 0) - mat(0, 1);
		double denom = std::sqrt(m21 * m21 + m02 * m02 + m10 * m10);
		out_axis[0] = m21 / denom;
		out_axis[1] = m02 / denom;
		out_axis[2] = m10 / denom;
		out_axis[3] = 0;
	}
}

void cMathUtil::DeltaRot(const tVector& axis0, double theta0, const tVector& axis1, double theta1,
							tVector& out_axis, double& out_theta)
{
	tMatrix R0 = RotateMat(axis0, theta0);
	tMatrix R1 = RotateMat(axis1, theta1);
	tMatrix M = DeltaRot(R0, R1);
	RotMatToAxisAngle(M, out_axis, out_theta);
}

tMatrix cMathUtil::DeltaRot(const tMatrix& R0, const tMatrix& R1)
{
	return R1 * R0.transpose();
}

double cMathUtil::Sign(double val)
{
	return SignAux<double>(val);
}

int cMathUtil::Sign(int val)
{
	return SignAux<int>(val);
}

double cMathUtil::AddAverage(double avg0, int count0, double avg1, int count1)
{
	double total = count0 + count1;
	double new_avg = (count0 * avg0 + count1 * avg1) / total;
	return new_avg;
}

void cMathUtil::CalcSoftmax(const Eigen::VectorXd& vals, double temp, Eigen::VectorXd& out_prob)
{
	assert(out_prob.size() == vals.size());
	int num_vals = static_cast<int>(vals.size());
	double sum = 0;
	double max_val = vals.maxCoeff();
	for (int i = 0; i < num_vals; ++i)
	{
		double val = vals[i];
		val = std::exp((val - max_val) / temp);
		out_prob[i] = val;
		sum += val;
	}

	for (int i = 0; i < num_vals; ++i)
	{
		double val = out_prob[i];
		double p = val / sum;
		out_prob[i] = p;
	}
}
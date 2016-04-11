#pragma once

#include <random>

#include "Eigen/Dense"
#include "Eigen/StdVector"
#include "Rand.h"

const int gInvalidIdx = -1;

// for convenience define standard vector for rendering
typedef Eigen::Vector4d tVector;
typedef Eigen::Vector4d tVector3;
typedef Eigen::Matrix4d tMatrix;
typedef Eigen::Matrix3d tMatrix3;
typedef std::vector<tVector, Eigen::aligned_allocator<tVector>> tVectorArr;

const double gRadiansToDegrees = 57.2957795;
const tVector gGravity = tVector(0, -9.8, 0, 0);

class cMathUtil
{
public:
	static int Clamp(int val, int min, int max);
	static double Clamp(double val, double min, double max);
	static double Saturate(double val);

	// rand number
	static double RandDouble();
	static double RandDouble(double min, double max);
	static double RandDoubleNorm(double mean, double stdev);
	static int RandInt(int min, int max);
	static int RandIntExclude(int min, int max, int exc);
	static void SeedRand(unsigned long int seed);
	static int RandSign();
	static double SmoothStep(double t);
	static bool FlipCoin(double p = 0.5);

	// matrices
	static tMatrix TranslateMat(const tVector& trans);
	static tMatrix ScaleMat(double scale);
	static tMatrix ScaleMat(const tVector& scale);
	static tMatrix RotateMat(const tVector& axis, double theta);
	static tMatrix CrossMat(const tVector& a);
	// inverts a transformation consisting only of rotations and translations
	static tMatrix InvRigidMat(const tMatrix& mat);
	static void RotMatToAxisAngle(const tMatrix& mat, tVector& out_axis, double& out_theta);

	static void DeltaRot(const tVector& axis0, double theta0, const tVector& axis1, double theta1,
							tVector& out_axis, double& out_theta);
	static tMatrix DeltaRot(const tMatrix& R0, const tMatrix& R1);

	static double Sign(double val);
	static int Sign(int val);

	static double AddAverage(double avg0, int count0, double avg1, int count1);
	static void CalcSoftmax(const Eigen::VectorXd& vals, double temp, Eigen::VectorXd& out_prob);

private:
	static cRand gRand;

	template <typename T>
	static T SignAux(T val)
	{
		return (T(0) < val) - (val < T(0));
	}
};

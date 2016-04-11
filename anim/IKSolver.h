#pragma once

#include "Eigen/Dense"
#include <random>
#include <time.h>
#include "util/MathUtil.h"

class cIKSolver
{
public:
	struct tProblem
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tProblem();
		~tProblem();

		Eigen::MatrixXd mJointDesc; // description of joint tree
		Eigen::VectorXd mPose;
		Eigen::MatrixXd mConsDesc; // constraints
		int mMaxIter;
		double mTol; // solution will consider to have converged if the change in the objective value is below tolerance
		double mDamp; // damping factor for damped least squares
		double mClampDist; // clamp maximum distance to target in order to reduce oscillation
		double mRestStateGain;
	};

	struct tSolution
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			Eigen::VectorXd mState;
	};

	// constraint types
	enum eConsType
	{
		eConsTypePos,
		eConsTypePosX,
		eConsTypePosY,
		eConsTypeTheta,
		eConsTypeThetaWorld,
		eConsTypeMax
	};

	enum eConsDesc
	{
		eConsDescType,
		eConsDescPriority,
		eConsDescWeight,
		eConsDescParam0,
		eConsDescParam1,
		eConsDescParam2,
		eConsDescParam3,
		eConsDescParam4,
		eConsDescMax
	};

	typedef Eigen::Matrix<double, eConsDescMax, 1> tConsDesc;

	static void PrintMatrix(const Eigen::MatrixXd& mat);
	static const int gInvalidJointID;
	static void Solve(const tProblem& prob, tSolution& out_soln);

	static Eigen::MatrixXd BuildJacob(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const Eigen::MatrixXd& cons_mat);
	static Eigen::MatrixXd BuildJacob(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc);
	static Eigen::VectorXd BuildErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const Eigen::MatrixXd& cons_mat);

	static double CalcObjVal(const Eigen::MatrixXd &joint_desc, const Eigen::VectorXd& pose, const Eigen::MatrixXd& cons_desc);

private:
	static std::default_random_engine gRandGen;
	static std::uniform_real_distribution<double> gRandDoubleDist;


	static void StepWeighted(const Eigen::MatrixXd& cons_desc, const tProblem& prob, Eigen::MatrixXd& joint_desc,
		Eigen::VectorXd& out_pose);
	static void StepHybrid(const Eigen::MatrixXd& cons_desc, const tProblem& prob, Eigen::MatrixXd& joint_desc,
		Eigen::VectorXd& out_pose);

	static Eigen::VectorXd BuildErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc);
	static Eigen::VectorXd BuildErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc, double clamp_dist);
	static Eigen::VectorXd BuildConsPosErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc, double clamp_dist);
	static Eigen::VectorXd BuildConsPosXErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc, double clamp_dist);
	static Eigen::VectorXd BuildConsPosYErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc, double clamp_dist);
	static Eigen::VectorXd BuildConsThetaErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc);
	static Eigen::VectorXd BuildConsThetaWorldErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc);

	static Eigen::MatrixXd BuildKernel(const Eigen::MatrixXd& mat);

	static Eigen::MatrixXd BuildConsPosJacob(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc);
	static Eigen::MatrixXd BuildConsPosXJacob(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc);
	static Eigen::MatrixXd BuildConsPosYJacob(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc);
	static Eigen::MatrixXd BuildConsThetaJacob(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc);
	static Eigen::MatrixXd BuildConsThetaWorldJacob(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc);

	static int CountConsDim(const Eigen::MatrixXd& cons_mat);
	static int GetConsDim(const tConsDesc& cons_desc);

	static void ClampMag(tVector& vec, double max_d);

};
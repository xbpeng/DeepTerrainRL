#pragma once

#include "sim/PDController.h"
#include "sim/RBDModel.h"

//#define IMP_PD_CTRL_PROFILER

class cImpPDController : public cController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cImpPDController();
	virtual ~cImpPDController();

	virtual void Init(cSimCharacter* character, const Eigen::MatrixXd& pd_params, const tVector& gravity);
	virtual void Init(cSimCharacter* character, const std::shared_ptr<cRBDModel>& model, const Eigen::MatrixXd& pd_params, const tVector& gravity);
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_step);
	virtual void UpdateControlForce(double time_step, Eigen::VectorXd& out_tau);

	virtual int GetNumJoints() const;
	virtual int GetNumDof() const;

	virtual double GetTargetTheta(int joint_id) const;
	virtual void SetTargetTheta(int joint_id, double theta);
	virtual void SetTargetVel(int joint_id, double vel);
	virtual bool UseWorldCoord(int joint_id) const;
	virtual void SetUseWorldCoord(int joint_id, bool use);
	virtual void SetKp(int joint_id, double kp);
	virtual void SetKd(int joint_id, double kd);
	virtual bool IsValidPDCtrl(int joint_id) const;

	virtual cPDController& GetPDCtrl(int joint_id);
	virtual const cPDController& GetPDCtrl(int joint_id) const;

protected:
	Eigen::VectorXd mKp;
	Eigen::VectorXd mKd;
	std::vector<cPDController, Eigen::aligned_allocator<cPDController>> mPDCtrls;

	tVector mGravity;
	bool mExternRBDModel;
	std::shared_ptr<cRBDModel> mRBDModel;

#if defined(IMP_PD_CTRL_PROFILER)
	double mPerfSolveTime;
	double mPerfTotalTime;
	int mPerfSolveCount;
	int mPerfTotalCount;
#endif // IMP_PD_CTRL_PROFILER

	virtual void InitGains();
	virtual std::shared_ptr<cRBDModel> BuildRBDModel(const cSimCharacter& character, const tVector& gravity) const;
	virtual void UpdateRBDModel();

	virtual void CalcControlForces(double time_step, Eigen::VectorXd& out_tau);
	virtual void BuildPoseErr(Eigen::VectorXd& out_pose_err) const;
	virtual void BuildVelErr(Eigen::VectorXd& out_vel_err) const;

	virtual void ApplyControlForces(const Eigen::VectorXd& tau);
};
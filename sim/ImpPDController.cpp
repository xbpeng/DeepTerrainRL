#include "ImpPDController.h"
#include <iostream>

#include "util/Util.h"
#include "sim/SimCharacter.h"
#include "sim/RBDUtil.h"

cImpPDController::cImpPDController()
{
	mExternRBDModel = true;

#if defined(IMP_PD_CTRL_PROFILER)
	mPerfSolveTime = 0;
	mPerfTotalTime = 0;
	mPerfSolveCount = 0;
	mPerfTotalCount = 0;
#endif // IMP_PD_CTRL_PROFILER
}

cImpPDController::~cImpPDController()
{
}

void cImpPDController::Init(cSimCharacter* character, const Eigen::MatrixXd& pd_params, const tVector& gravity)
{
	std::shared_ptr<cRBDModel> model = BuildRBDModel(*character, gravity);
	Init(character, model, pd_params, gravity);
	mExternRBDModel = false;
}

void cImpPDController::Init(cSimCharacter* character, const std::shared_ptr<cRBDModel>& model, const Eigen::MatrixXd& pd_params, const tVector& gravity)
{
	cController::Init(character);
	int num_joints = mChar->GetNumJoints();
	mPDCtrls.resize(num_joints);
	assert(pd_params.rows() == num_joints);

	mGravity = gravity;
	mRBDModel = model;

	int idx = 0;
	for (int j = 0; j < num_joints; ++j)
	{
		cJoint& joint = mChar->GetJoint(j);
		if (joint.IsValid())
		{
			const cPDController::tParams& curr_params = pd_params.row(j);
			cPDController& ctrl = mPDCtrls[j];
			ctrl.Init(mChar, curr_params);
		}
	}

	InitGains();
	mValid = true;
}

void cImpPDController::Reset()
{
	cController::Reset();
	for (size_t i = 0; i < mPDCtrls.size(); ++i)
	{
		mPDCtrls[i].Reset();
	}
}

void cImpPDController::Clear()
{
	cController::Clear();
	mPDCtrls.clear();
	mExternRBDModel = true;
	mRBDModel.reset();
}

void cImpPDController::Update(double time_step)
{
	Eigen::VectorXd tau = Eigen::VectorXd::Zero(GetNumDof());
	UpdateControlForce(time_step, tau);
	ApplyControlForces(tau);
}

void cImpPDController::UpdateControlForce(double time_step, Eigen::VectorXd& out_tau)
{
	cController::Update(time_step);

#if defined(IMP_PD_CTRL_PROFILER)
	TIMER_RECORD_BEG(Update_Ctrl_Force)
#endif

	if (time_step > 0)
	{
		if (!mExternRBDModel)
		{
			UpdateRBDModel();
		}

		Eigen::VectorXd tau;
		CalcControlForces(time_step, tau);
		out_tau += tau;
	}

#if defined(IMP_PD_CTRL_PROFILER)
	TIMER_RECORD_END(Update_Ctrl_Force, mPerfTotalTime, mPerfTotalCount)
#endif

#if defined(IMP_PD_CTRL_PROFILER)
	printf("Solve Time: %.5f\n", mPerfSolveTime);
	printf("Total Time: %.5f\n", mPerfTotalTime);
#endif
}

int cImpPDController::GetNumJoints() const
{
	return mChar->GetNumJoints();
}

int cImpPDController::GetNumDof() const
{
	return mChar->GetNumDof();
}

double cImpPDController::GetTargetTheta(int joint_id) const
{
	const cPDController& pd_ctrl = GetPDCtrl(joint_id);
	return pd_ctrl.GetTargetTheta();
}

void cImpPDController::SetTargetTheta(int joint_id, double theta)
{
	cPDController& pd_ctrl = GetPDCtrl(joint_id);
	pd_ctrl.SetTargetTheta(theta);
}

void cImpPDController::SetTargetVel(int joint_id, double vel)
{
	cPDController& pd_ctrl = GetPDCtrl(joint_id);
	pd_ctrl.SetTargetVel(vel);
}

bool cImpPDController::UseWorldCoord(int joint_id) const
{
	const cPDController& pd_ctrl = GetPDCtrl(joint_id);
	return pd_ctrl.UseWorldCoord();
}

void cImpPDController::SetUseWorldCoord(int joint_id, bool use)
{
	cPDController& pd_ctrl = GetPDCtrl(joint_id);
	pd_ctrl.SetUseWorldCoord(use);
}

void cImpPDController::SetKp(int joint_id, double kp)
{
	cPDController& pd_ctrl = GetPDCtrl(joint_id);
	pd_ctrl.SetKp(kp);

	int param_offset = mChar->GetParamOffset(joint_id);
	int param_size = mChar->GetParamSize(joint_id);

	auto curr_kp = mKp.segment(param_offset, param_size);
	curr_kp.setOnes();
	curr_kp *= kp;
}

void cImpPDController::SetKd(int joint_id, double kd)
{
	cPDController& pd_ctrl = GetPDCtrl(joint_id);
	pd_ctrl.SetKd(kd);

	int param_offset = mChar->GetParamOffset(joint_id);
	int param_size = mChar->GetParamSize(joint_id);

	auto curr_kd = mKp.segment(param_offset, param_size);
	curr_kd.setOnes();
	curr_kd *= kd;
}

bool cImpPDController::IsValidPDCtrl(int joint_id) const
{
	const cPDController& pd_ctrl = GetPDCtrl(joint_id);
	return pd_ctrl.IsValid();
}

void cImpPDController::InitGains()
{
	int num_dof = GetNumDof();
	mKp = Eigen::VectorXd::Zero(num_dof);
	mKd = Eigen::VectorXd::Zero(num_dof);

	for (int j = 0; j < GetNumJoints(); ++j)
	{
		const cPDController& pd_ctrl = GetPDCtrl(j);
		if (pd_ctrl.IsValid())
		{
			int param_offset = mChar->GetParamOffset(j);
			int param_size = mChar->GetParamSize(j);

			double kp = pd_ctrl.GetKp();
			double kd = pd_ctrl.GetKd();

			mKp.segment(param_offset, param_size) = Eigen::VectorXd::Ones(param_size) * kp;
			mKd.segment(param_offset, param_size) = Eigen::VectorXd::Ones(param_size) * kd;
		}
	}
}

std::shared_ptr<cRBDModel> cImpPDController::BuildRBDModel(const cSimCharacter& character, const tVector& gravity) const
{
	std::shared_ptr<cRBDModel> model = std::shared_ptr<cRBDModel>(new cRBDModel());
	model->Init(character.GetJointMat(), character.GetBodyDefs(), gravity);
	return model;
}

void cImpPDController::UpdateRBDModel()
{
	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	mChar->BuildPose(pose);
	mChar->BuildVel(vel);
	mRBDModel->Update(pose, vel);
}

cPDController& cImpPDController::GetPDCtrl(int joint_id)
{
	assert(joint_id >= 0 && joint_id < GetNumJoints());
	return mPDCtrls[joint_id];
}

const cPDController& cImpPDController::GetPDCtrl(int joint_id) const
{
	assert(joint_id >= 0 && joint_id < GetNumJoints());
	return mPDCtrls[joint_id];
}

void cImpPDController::CalcControlForces(double time_step, Eigen::VectorXd& out_tau)
{
	double t = time_step;

	Eigen::VectorXd pose_err;
	Eigen::VectorXd vel_err;
	BuildPoseErr(pose_err);
	BuildVelErr(vel_err);

	Eigen::DiagonalMatrix<double, Eigen::Dynamic> Kp_mat = mKp.asDiagonal();
	Eigen::DiagonalMatrix<double, Eigen::Dynamic> Kd_mat = mKd.asDiagonal();

	for (int j = 0; j < GetNumJoints(); ++j)
	{
		const cPDController& pd_ctrl = GetPDCtrl(j);
		if (!pd_ctrl.IsValid() || !pd_ctrl.IsActive())
		{
			int param_offset = mChar->GetParamOffset(j);
			int param_size = mChar->GetParamSize(j);
			Kp_mat.diagonal().segment(param_offset, param_size).setZero();
			Kd_mat.diagonal().segment(param_offset, param_size).setZero();
		}
	}

	Eigen::MatrixXd M = mRBDModel->GetMassMat();
	const Eigen::VectorXd& C = mRBDModel->GetBiasForce();

	M.diagonal() += t * mKd;

	const Eigen::VectorXd& vel = mRBDModel->GetVel();
	Eigen::VectorXd acc;
	acc = Kp_mat * (pose_err - t * vel) + Kd_mat * vel_err - C;
	
#if defined(IMP_PD_CTRL_PROFILER)
	TIMER_RECORD_BEG(Solve)
#endif

	acc = M.ldlt().solve(acc);

#if defined(IMP_PD_CTRL_PROFILER)
	TIMER_RECORD_END(Solve, mPerfSolveTime, mPerfSolveCount)
#endif
	
	out_tau = Kp_mat * (pose_err - t * vel) + Kd_mat * (vel_err - t * acc);
}

void cImpPDController::BuildPoseErr(Eigen::VectorXd& out_pose_err) const
{
	out_pose_err = Eigen::VectorXd::Zero(GetNumDof());
	for (int j = 0; j < GetNumJoints(); ++j)
	{
		const cPDController& pd_ctrl = GetPDCtrl(j);
		if (pd_ctrl.IsValid())
		{
			double curr_err = pd_ctrl.CalcThetaErr();
			int param_offset = mChar->GetParamOffset(j);
			int param_size = mChar->GetParamSize(j);
			out_pose_err.segment(param_offset, param_size) = Eigen::VectorXd::Ones(param_size) * curr_err;
		}
	}
}

void cImpPDController::BuildVelErr(Eigen::VectorXd& out_vel_err) const
{
	out_vel_err = Eigen::VectorXd::Zero(GetNumDof());
	for (int j = 0; j < GetNumJoints(); ++j)
	{
		const cPDController& pd_ctrl = GetPDCtrl(j);
		if (pd_ctrl.IsValid())
		{
			double curr_err = pd_ctrl.CalcVelErr();
			int param_offset = mChar->GetParamOffset(j);
			int param_size = mChar->GetParamSize(j);
			out_vel_err.segment(param_offset, param_size) = Eigen::VectorXd::Ones(param_size) * curr_err;
		}
	}
}

void cImpPDController::ApplyControlForces(const Eigen::VectorXd& tau)
{
	mChar->ApplyControlForces(tau);
}
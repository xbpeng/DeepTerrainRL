#include "KinCharacter.h"
#include <assert.h>

const double gDiffTimeStep = 1 / 60.0;

cKinCharacter::cKinCharacter()
{
	mOrigin.setZero();
	mCycleRootDelta.setZero();
}

cKinCharacter::~cKinCharacter()
{

}

bool cKinCharacter::Init(const std::string& char_file, const std::string& motion_file)
{
	bool succ = Init(char_file);
	if (succ && (motion_file != ""))
	{
		bool succ_motion = LoadMotion(motion_file);
		if (!succ_motion)
		{
			printf("Failed to load motion from %s\n", motion_file.c_str());
		}
		succ &= succ_motion;
	}
	

	return succ;
}

bool cKinCharacter::Init(const std::string& char_file)
{
	return cCharacter::Init(char_file);
}

void cKinCharacter::Clear()
{
	cCharacter::Clear();
	mMotion.Clear();
}

void cKinCharacter::Update(double time_step)
{
	cCharacter::Update(time_step);
	mTime += time_step;

	if (mMotion.IsValid())
	{
		Pose(mTime);
	}
}

void cKinCharacter::Reset()
{
	cCharacter::Reset();
}

const cMotion& cKinCharacter::GetMotion() const
{
	return mMotion;
}

double cKinCharacter::GetMotionDuration() const
{
	if (mMotion.IsValid())
	{
		return mMotion.GetDuration();
	}
	return 0;
}

void cKinCharacter::SetTime(double time)
{
	mTime = time;
}

double cKinCharacter::GetTime() const
{
	return mTime;
}

void cKinCharacter::Pose(double time)
{
	Eigen::VectorXd pose;
	CalcPose(time, pose);
	SetPose(pose);
}

void cKinCharacter::BuildPose(Eigen::VectorXd& out_pose) const
{
	cCharacter::BuildPose(out_pose);

	const Eigen::MatrixXd& joint_desc = GetJointMat();
	tVector root_pos = cKinTree::GetRootPos(joint_desc, out_pose);
	root_pos += GetOrigin();
	cKinTree::SetRootPos(joint_desc, root_pos, out_pose);
}


void cKinCharacter::BuildVel(Eigen::VectorXd& out_vel) const
{
	// approximate velocity with finite difference
	Eigen::VectorXd curr_pose;
	BuildPose(curr_pose);

	Eigen::VectorXd new_pose;
	CalcPose(mTime + gDiffTimeStep, new_pose);

	out_vel = new_pose - curr_pose;
	out_vel /= gDiffTimeStep;
}

void cKinCharacter::BuildAcc(Eigen::VectorXd& out_acc) const
{
	// approximate velocity with finite difference
	Eigen::VectorXd pose0;
	Eigen::VectorXd pose1;
	Eigen::VectorXd pose2;

	BuildPose(pose1);
	CalcPose(mTime - gDiffTimeStep, pose0);
	CalcPose(mTime + gDiffTimeStep, pose2);

	out_acc = pose2 - 2 * pose1 + pose0;
	out_acc /= gDiffTimeStep * gDiffTimeStep;
}

void cKinCharacter::SetPose(const Eigen::VectorXd& pose)
{
	Eigen::VectorXd offset_pose = pose;
	const Eigen::MatrixXd& joint_desc = GetJointMat();
	tVector root_pos = cKinTree::GetRootPos(joint_desc, offset_pose);
	root_pos -= GetOrigin();
	cKinTree::SetRootPos(joint_desc, root_pos, offset_pose);
	cCharacter::SetPose(offset_pose);
}

void cKinCharacter::BuildPose0(Eigen::VectorXd& out_pose) const
{
	cCharacter::BuildPose0(out_pose);
	tVector root_pos = cKinTree::GetRootPos(mJointMat, out_pose);
	root_pos += GetOrigin();
	cKinTree::SetRootPos(mJointMat, root_pos, out_pose);
}

void cKinCharacter::SetPose0(const Eigen::VectorXd& pose)
{
	Eigen::VectorXd offset_pose = pose;
	const Eigen::MatrixXd& joint_desc = GetJointMat();
	tVector root_pos = cKinTree::GetRootPos(joint_desc, offset_pose);
	root_pos -= GetOrigin();
	cKinTree::SetRootPos(joint_desc, root_pos, offset_pose);
	cCharacter::SetPose0(offset_pose);
}

bool cKinCharacter::HasMotion() const
{
	return mMotion.IsValid();
}

tVector cKinCharacter::GetRootPos() const
{
	return cCharacter::GetRootPos() + GetOrigin();
}

const tVector& cKinCharacter::GetOrigin() const
{
	return mOrigin;
}

void cKinCharacter::SetOrigin(const tVector& origin)
{
	tVector delta = origin - mOrigin;
	MoveOrigin(delta);
}

void cKinCharacter::MoveOrigin(const tVector& delta)
{
	mOrigin += delta;
}

void cKinCharacter::ResetParams()
{ 
	cCharacter::ResetParams();
	mTime = 0;
}

bool cKinCharacter::LoadMotion(const std::string& motion_file)
{
	bool succ = mMotion.Load(motion_file);

	if (succ)
	{
		int char_dof = GetNumDof();
		int motion_dof = mMotion.GetNumDof();

		if (char_dof != motion_dof)
		{
			printf("DOF mismatch, char dof: %i, motion dof: %i\n", char_dof, motion_dof);
			mMotion.Clear();
			succ = false;
		}
	}

	if (succ)
	{
		mCycleRootDelta = CalcCycleRootDelta();
		Pose(mTime);
		BuildPose(mPose0);
		BuildVel(mVel0);
	}

	return succ;
}

tVector cKinCharacter::CalcCycleRootDelta() const
{
	int num_frames = mMotion.GetNumFrames();
	Eigen::VectorXd frame_beg = mMotion.GetFrame(0);
	Eigen::VectorXd  frame_end = mMotion.GetFrame(num_frames - 1);

	tVector root_pos_beg = cKinTree::GetRootPos(mJointMat, frame_beg);
	tVector root_pos_end = cKinTree::GetRootPos(mJointMat, frame_end);

	tVector delta = root_pos_end - root_pos_beg;
	return delta;
}

void cKinCharacter::CalcPose(double time, Eigen::VectorXd& out_pose) const
{
	out_pose = mMotion.CalcFrame(time);
	int cycle_count = mMotion.CalcCycleCount(time);
	tVector root_delta = cycle_count * mCycleRootDelta;
	root_delta += mOrigin;

	tVector root_pos = cKinTree::GetRootPos(mJointMat, out_pose);
	root_pos += root_delta;
	cKinTree::SetRootPos(mJointMat, root_pos, out_pose);
}
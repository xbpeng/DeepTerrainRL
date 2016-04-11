#include "Character.h"
#include <assert.h>
#include <json/json.h>

#include "util/FileUtil.h"
#include "util/JsonUtil.h"

// Json keys
const std::string gSkeletonKey = "Skeleton";
const std::string gPoseKey = "Pose";
const std::string gVelKey = "Vel";

cCharacter::cCharacter()
{
	ResetParams();
}

cCharacter::~cCharacter()
{

}

bool cCharacter::Init(const std::string& char_file)
{
	Clear();

	bool succ = true;
	if (char_file != "")
	{
		std::ifstream f_stream(char_file);
		Json::Reader reader;
		Json::Value root;
		succ = reader.parse(f_stream, root);
		f_stream.close();

		if (succ)
		{
			if (root[gSkeletonKey].isNull())
			{
				succ = false;
			}
			else
			{
				succ = LoadSkeleton(root[gSkeletonKey]);
			}
		}
	}

	if (succ)
	{
		InitDefaultState();
	}

	if (!succ)
	{
		printf("Failed to parse character from file %s.\n", char_file.c_str());
	}

	return succ;
}

void cCharacter::Clear()
{
	ResetParams();
	mPose.resize(0);
	mVel.resize(0);
	mPose0.resize(0);
	mVel0.resize(0);
}

void cCharacter::Update(double time_step)
{
}

void cCharacter::Reset()
{
	ResetParams();

	Eigen::VectorXd pose0;
	Eigen::VectorXd vel0;
	BuildPose0(pose0);
	BuildVel0(vel0);

	SetPose(pose0);
	SetVel(vel0);
}

int cCharacter::GetNumDof() const
{
	int dofs = cKinTree::GetNumDof(mJointMat);
	return dofs;
}

const Eigen::MatrixXd& cCharacter::GetJointMat() const
{
	return mJointMat;
}

int cCharacter::GetNumJoints() const
{
	return cKinTree::GetNumJoints(mJointMat);
}

void cCharacter::BuildPose(Eigen::VectorXd& out_pose) const
{
	out_pose = mPose;
}

void cCharacter::SetPose(const Eigen::VectorXd& pose)
{
	assert(pose.size() == GetNumDof());
	mPose = pose;
}

void cCharacter::BuildVel(Eigen::VectorXd& out_vel) const
{
	out_vel = mVel;
}

void cCharacter::SetVel(const Eigen::VectorXd& vel)
{
	mVel = vel;
}

void cCharacter::SetPose0(const Eigen::VectorXd& pose)
{
	mPose0 = pose;
}

void cCharacter::SetVel0(const Eigen::VectorXd& vel)
{
	mVel0 = vel;
}

tVector cCharacter::GetRootPos() const
{
	tVector pos = cKinTree::GetRootPos(mJointMat, mPose);
	return pos;
}

void cCharacter::GetRootRotation(tVector& out_axis, double& out_theta) const
{
	out_axis = tVector(0, 0, 1, 0);
	out_theta = cKinTree::GetJointTheta(mJointMat, mPose, GetRootID());
}

int cCharacter::GetRootID() const
{
	int root_id = cKinTree::GetRoot(mJointMat);
	return root_id;
}

int cCharacter::GetParamOffset(int joint_id) const
{
	return cKinTree::GetParamOffset(mJointMat, joint_id);
}

int cCharacter::GetParamSize(int joint_id) const
{
	return cKinTree::GetParamSize(mJointMat, joint_id);
}

tVector cCharacter::CalcJointPos(int joint_id) const
{
	tVector pos = cKinTree::CalcJointWorldPos(mJointMat, mPose, joint_id);
	return pos;
}

tVector cCharacter::CalcJointVel(int joint_id) const
{
	tVector pos = cKinTree::CalcJointWorldVel(mJointMat, mPose, mVel, joint_id);
	return pos;
}

void cCharacter::CalcJointWorldRotation(int joint_id, tVector& out_axis, double& out_theta) const
{
	cKinTree::CalcJointWorldTheta(mJointMat, mPose, joint_id, out_axis, out_theta);
}

void cCharacter::CalcJointRotation(int joint_id, tVector& out_axis, double& out_theta) const
{
	out_theta = cKinTree::GetJointTheta(mJointMat, mPose, joint_id);
	out_axis = tVector(0, 0, 1, 0);
}

double cCharacter::CalcJointChainLength(int joint_id)
{
	auto chain = cKinTree::FindJointChain(mJointMat, GetRootID(), joint_id);
	return cKinTree::CalcChainLength(mJointMat, chain);
}

tMatrix cCharacter::BuildJointWorldTrans(int joint_id) const
{
	return cKinTree::JointWorldTrans(mJointMat, mPose, joint_id);
}

void cCharacter::CalcAABB(tVector& out_min, tVector& out_max) const
{
	cKinTree::CalcAABB(mJointMat, mPose, out_min, out_max);
}

void cCharacter::BuildPose0(Eigen::VectorXd& out_pose) const
{
	out_pose = mPose0;
}

void cCharacter::BuildVel0(Eigen::VectorXd& out_vel) const
{
	out_vel = mVel0;
}

bool cCharacter::WriteState(const std::string& file) const
{
	return WriteState(file, tVector::Zero());
}

bool cCharacter::WriteState(const std::string& file, const tVector& root_offset) const
{
	Eigen::VectorXd pose;
	Eigen::VectorXd vel;
	BuildPose(pose);
	BuildVel(vel);

	tVector root_pos = cKinTree::GetRootPos(mJointMat, pose);
	root_pos += root_offset;
	cKinTree::SetRootPos(mJointMat, root_pos, pose);

	std::string json = BuildStateJson(pose, vel);
	FILE* f = cFileUtil::OpenFile(file, "w");
	if (f != nullptr)
	{
		fprintf(f, "%s", json.c_str());
		cFileUtil::CloseFile(f);
		return true;
	}
	return false;
}

bool cCharacter::ReadState(const std::string& file)
{
	std::ifstream f_stream(file);
	Json::Reader reader;
	Json::Value root;
	bool succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ && !root[gPoseKey].isNull())
	{
		Eigen::VectorXd pose;
		succ &= ParseState(root[gPoseKey], pose);
		SetPose(pose);
	}

	if (succ && !root[gVelKey].isNull())
	{
		Eigen::VectorXd vel;
		succ &= ParseState(root[gVelKey], vel);
		SetVel(vel);
	}

	return succ;
}

bool cCharacter::LoadSkeleton(const Json::Value& root)
{
	return cKinTree::Load(root, mJointMat);
}

void cCharacter::InitDefaultState()
{
	int state_size = GetNumDof();
	mPose0 = Eigen::VectorXd::Zero(state_size);
	mVel0 = Eigen::VectorXd::Zero(state_size);
	mPose = mPose0;
	mVel = mVel0;
}

void cCharacter::RecordDefaultState()
{
	BuildPose(mPose0);
	BuildVel(mVel0);
}

void cCharacter::ResetParams()
{
}

bool cCharacter::ParseState(const Json::Value& root, Eigen::VectorXd& out_state) const
{
	bool succ = cJsonUtil::ReadVectorJson(root, out_state);
	assert(out_state.size() == GetNumDof());
	return succ;
}

std::string cCharacter::BuildStateJson(const Eigen::VectorXd& pose, const Eigen::VectorXd& vel) const
{
	std::string json = "";

	std::string pose_json = cJsonUtil::BuildVectorJson(pose);
	std::string vel_json = cJsonUtil::BuildVectorJson(vel);

	json = "{\n\"Pose\":" + pose_json + ",\n\"Vel\":" + vel_json + "\n}";
	return json;
}

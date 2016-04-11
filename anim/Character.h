#pragma once

#include "KinTree.h"

class cCharacter
{
public:
	virtual ~cCharacter();

	virtual bool Init(const std::string& char_file);
	virtual void Clear();
	virtual void Update(double time_step);
	virtual void Reset();

	virtual int GetNumDof() const;
	virtual const Eigen::MatrixXd& GetJointMat() const;
	virtual int GetNumJoints() const;
	virtual void BuildPose(Eigen::VectorXd& out_pose) const;
	virtual void SetPose(const Eigen::VectorXd& pose);
	virtual void BuildVel(Eigen::VectorXd& out_state) const;
	virtual void SetVel(const Eigen::VectorXd& vel);
	virtual void SetPose0(const Eigen::VectorXd& pose);
	virtual void SetVel0(const Eigen::VectorXd& vel);

	virtual tVector GetRootPos() const;
	virtual void GetRootRotation(tVector& out_axis, double& out_theta) const;
	virtual int GetRootID() const;

	virtual int GetParamOffset(int joint_id) const;
	virtual int GetParamSize(int joint_id) const;

	virtual tVector CalcJointPos(int joint_id) const;
	virtual tVector CalcJointVel(int joint_id) const;
	virtual void CalcJointWorldRotation(int joint_id, tVector& out_axis, double& out_theta) const;
	virtual void CalcJointRotation(int joint_id, tVector& out_axis, double& out_theta) const;
	virtual double CalcJointChainLength(int joint_id);
	virtual tMatrix BuildJointWorldTrans(int joint_id) const;

	virtual void CalcAABB(tVector& out_min, tVector& out_max) const;

	virtual void BuildPose0(Eigen::VectorXd& out_pose) const;
	virtual void BuildVel0(Eigen::VectorXd& out_vel) const;

	virtual bool WriteState(const std::string& file) const;
	virtual bool WriteState(const std::string& file, const tVector& root_offset) const;
	virtual bool ReadState(const std::string& file);

protected:
	Eigen::MatrixXd mJointMat;
	Eigen::VectorXd mPose;
	Eigen::VectorXd mVel;
	Eigen::VectorXd mPose0;
	Eigen::VectorXd mVel0;

	cCharacter();

	virtual bool LoadSkeleton(const Json::Value& root);
	virtual void InitDefaultState();
	virtual void RecordDefaultState();
	virtual void ResetParams();
	virtual bool ParseState(const Json::Value& root, Eigen::VectorXd& out_state) const;
	virtual std::string BuildStateJson(const Eigen::VectorXd& pose, const Eigen::VectorXd& vel) const;
};
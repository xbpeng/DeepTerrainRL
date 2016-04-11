#pragma once

#include "Character.h"
#include "Motion.h"

class cKinCharacter : public cCharacter
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cKinCharacter();
	virtual ~cKinCharacter();

	virtual bool Init(const std::string& char_file, const std::string& motion_file);
	virtual bool Init(const std::string& char_file);
	virtual void Clear();
	virtual void Update(double time_step);
	virtual void Reset();

	virtual const cMotion& GetMotion() const;
	virtual double GetMotionDuration() const;
	virtual void SetTime(double time);
	virtual double GetTime() const;

	virtual void Pose(double time);
	virtual void BuildPose(Eigen::VectorXd& out_pose) const;
	virtual void BuildVel(Eigen::VectorXd& out_vel) const;
	virtual void BuildAcc(Eigen::VectorXd& out_acc) const;
	virtual void BuildPose0(Eigen::VectorXd& out_pose) const;
	virtual void SetPose(const Eigen::VectorXd& pose);
	virtual void SetPose0(const Eigen::VectorXd& pose);
	
	virtual bool HasMotion() const;

	virtual tVector GetRootPos() const;
	virtual const tVector& GetOrigin() const;
	virtual void SetOrigin(const tVector& origin);
	virtual void MoveOrigin(const tVector& delta);

	virtual void CalcPose(double time, Eigen::VectorXd& out_pose) const;

protected:
	double mTime;
	cMotion mMotion;

	tVector mCycleRootDelta;
	tVector mOrigin;

	virtual void ResetParams();
	virtual bool LoadMotion(const std::string& motion_file);
	virtual tVector CalcCycleRootDelta() const;
};
#pragma once

#include "sim/SimCharSoftFall.h"


class cSimDog : public cSimCharSoftFall
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eJoint
	{
		eJointRoot,
		eJointSpine0,
		eJointSpine1,
		eJointSpine2,
		eJointSpine3,
		eJointTorso,
		eJointNeck0,
		eJointNeck1,
		eJointHead,
		eJointTail0,
		eJointTail1,
		eJointTail2,
		eJointTail3,
		eJointShoulder,
		eJointElbow,
		eJointWrist,
		eJointFinger,
		eJointHip,
		eJointKnee,
		eJointAnkle,
		eJointToe,
		eJointMax,
		eJointInvalid
	};

	cSimDog();
	virtual ~cSimDog();

	virtual double GetPoseJointWeight(int joint_id) const;
	virtual bool HasStumbled() const;

	virtual bool IsEndEffector(int idx) const;

protected:
	virtual short GetPartColGroup(int part_id) const;
	virtual short GetPartColMask(int part_id) const;

	virtual bool CheckFallContact() const;
	virtual bool FailFallMisc() const;
};
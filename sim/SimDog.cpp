#include "sim/SimDog.h"
#include <iostream>

// bit 0 is used for filtering raytest
const short gColFlagBody = 1 << 1;
const short gColFlagFrontLeg = 1 << 2;
const short gColFlagBackLeg = 1 << 3;
const short gColFlagTail = cContactManager::gFlagNone; // tail is too small, so to prevent tunnelling disable all collisions

const short gBodyColFlags[cSimDog::eJointMax] =
{
	gColFlagBody, //eJointRoot,
	gColFlagBody, //eJointSpine0,
	gColFlagBody, //eJointSpine1,
	gColFlagBody, //eJointSpine2,
	gColFlagBody, //eJointSpine3,
	gColFlagBody, //eJointTorso,
	gColFlagBody, //eJointNeck0,
	gColFlagBody, //eJointNeck1,
	gColFlagBody, //eJointHead,
	gColFlagTail, //eJointTail0,
	gColFlagTail, //eJointTail1,
	gColFlagTail, //eJointTail2,
	gColFlagTail, //eJointTail3,
	gColFlagFrontLeg, //eJointShoulder,
	gColFlagFrontLeg, //eJointElbow,
	gColFlagFrontLeg, //eJointWrist,
	gColFlagFrontLeg, //eJointFinger,
	gColFlagBackLeg, //eJointHip,
	gColFlagBackLeg, //eJointKnee,
	gColFlagBackLeg, //eJointAnkle,
	gColFlagBackLeg, //eJointToe,
};

const double gPoseJointWeights[cSimDog::eJointMax] =
{
	1, //eJointRoot,
	0.5, //eJointSpine0,
	0.5, //eJointSpine1,
	0.5, //eJointSpine2,
	0.5, //eJointSpine3,
	1, //eJointTorso,
	0.001, //eJointNeck0,
	0.001, //eJointNeck1,
	0.001, //eJointHead,
	0.001, //eJointTail0,
	0.001, //eJointTail1,
	0.001, //eJointTail2,
	0.001, //eJointTail3,
	1, //eJointShoulder,
	1, //eJointElbow,
	1, //eJointWrist,
	0.1, //eJointFinger,
	1, //eJointHip,
	1, //eJointKnee,
	1, //eJointAnkle,
	0.1, //eJointToe,
};

cSimDog::cSimDog()
{
}

cSimDog::~cSimDog()
{
}

double cSimDog::GetPoseJointWeight(int joint_id) const
{
	return gPoseJointWeights[joint_id];
}

short cSimDog::GetPartColGroup(int part_id) const
{
	return gBodyColFlags[part_id];
}

short cSimDog::GetPartColMask(int part_id) const
{
	return gBodyColFlags[part_id];
}

bool cSimDog::HasStumbled() const
{
	bool stumbled = false;
	for (int i = 0; i < cSimDog::eJointMax; ++i)
	{
		cSimDog::eJoint joint_id = static_cast<cSimDog::eJoint>(i);

		if (joint_id != cSimDog::eJointToe 
			&& joint_id != cSimDog::eJointFinger
			&& joint_id != cSimDog::eJointAnkle
			&& joint_id != cSimDog::eJointWrist)
		{
			const auto& curr_part = GetBodyPart(joint_id);
			bool contact = curr_part->IsInContact();
			if (contact)
			{
				stumbled = true;
				break;
			}
		}
	}
	return stumbled;
}

bool cSimDog::IsEndEffector(int idx) const
{
	return (idx == eJointToe) || (idx == eJointFinger);
}

bool cSimDog::CheckFallContact() const
{
	const cSimDog::eJoint test_parts[] =
	{
		eJointRoot,
		eJointSpine0,
		eJointSpine1,
		eJointSpine2,
		eJointSpine3,
		eJointTorso,
		eJointNeck0,
		eJointNeck1,
		eJointHead
	};
	const int num_parts = sizeof(test_parts) / sizeof(test_parts[0]);

	bool fallen = false;
	for (int i = 0; i < num_parts; ++i)
	{
		cSimDog::eJoint joint_id = test_parts[i];
		const auto& curr_part = GetBodyPart(joint_id);
		bool contact = curr_part->IsInContact();
		if (contact)
		{
			fallen = true;
			break;
		}
	}

	return fallen;
}

bool cSimDog::FailFallMisc() const
{
	bool fallen = false;

	const tVector ref_axis = tVector(0, 0, 1, 0);
	tVector root_axis;
	double root_theta;
	GetRootRotation(root_axis, root_theta);

	if (root_axis.dot(ref_axis) < 0)
	{
		root_theta = -root_theta;
	}

	bool flipped = std::abs(root_theta) > M_PI * 0.8;
	fallen |= flipped;

	return fallen;
}
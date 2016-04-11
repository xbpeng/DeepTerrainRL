#include "sim/SimRaptor.h"
#include <iostream>

// bit 0 is used for filtering raytest
const short gColFlagBody = 1 << 1;
const short gColFlagLeg = 1 << 2;

const short gBodyColFlags[cSimRaptor::eJointMax] =
{
	gColFlagBody, // eJointRoot,
	gColFlagBody, // eJointSpine0,
	gColFlagBody, // eJointSpine1,
	gColFlagBody, // eJointSpine2,
	gColFlagBody, // eJointSpine3,
	gColFlagBody, // eJointHead,
	gColFlagBody, // eJointTail0,
	gColFlagBody, // eJointTail1,
	gColFlagBody, // eJointTail2,
	gColFlagBody, // eJointTail3,
	gColFlagBody, // eJointTail4,
	gColFlagLeg, // eJointRightHip,
	gColFlagLeg, // eJointRightKnee,
	gColFlagLeg, // eJointRightAnkle,
	gColFlagLeg, // eJointRightToe,
	gColFlagLeg, // eJointLeftHip,
	gColFlagLeg, // eJointLeftKnee,
	gColFlagLeg, // eJointLeftAnkle,
	gColFlagLeg, // eJointLeftToe,
};

const double gPoseJointWeights[cSimRaptor::eJointMax] =
{
	1, // eJointRoot,
	0.5, // eJointSpine0,
	0.5, // eJointSpine1,
	0.5, // eJointSpine2,
	0.5, // eJointSpine3,
	0.5, // eJointHead,
	0.01, // eJointTail0,
	0.01, // eJointTail1,
	0.01, // eJointTail2,
	0.01, // eJointTail3,
	0.01, // eJointTail4,
	1, // eJointRightHip,
	0.5, // eJointRightKnee,
	0.5, // eJointRightAnkle,
	0.2, // eJointRightToe,
	1, // eJointLeftHip,
	0.5, // eJointLeftKnee,
	0.5, // eJointLeftAnkle,
	0.2, // eJointLeftToe,
};

cSimRaptor::cSimRaptor()
{
	//mFriction = 0.9;
}

cSimRaptor::~cSimRaptor()
{
}

double cSimRaptor::GetPoseJointWeight(int joint_id) const
{
	return gPoseJointWeights[joint_id];
}

short cSimRaptor::GetPartColGroup(int part_id) const
{
	return gBodyColFlags[part_id];
}

short cSimRaptor::GetPartColMask(int part_id) const
{
	return gBodyColFlags[part_id];
}

bool cSimRaptor::HasStumbled() const
{
	bool stumbled = false;
	for (int i = 0; i < cSimRaptor::eJointMax; ++i)
	{
		cSimRaptor::eJoint joint_id = static_cast<cSimRaptor::eJoint>(i);

		if (joint_id != cSimRaptor::eJointRightToe 
			&& joint_id != cSimRaptor::eJointLeftToe
			&& joint_id != cSimRaptor::eJointRightAnkle
			&& joint_id != cSimRaptor::eJointLeftAnkle)
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

bool cSimRaptor::IsEndEffector(int idx) const
{
	return (idx == eJointRightToe) || (idx == eJointLeftToe);
}

bool cSimRaptor::CheckFallContact() const
{
	const cSimRaptor::eJoint test_parts[] =
	{
		eJointRoot,
		eJointSpine0,
		eJointSpine1,
		eJointSpine2,
		eJointSpine3,
		eJointHead,
	};
	const int num_parts = sizeof(test_parts) / sizeof(test_parts[0]);

	bool fallen = false;
	for (int i = 0; i < num_parts; ++i)
	{
		cSimRaptor::eJoint joint_id = test_parts[i];
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

bool cSimRaptor::FailFallMisc() const
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
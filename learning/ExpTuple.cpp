#include "ExpTuple.h"

bool tExpTuple::TestFlag(unsigned int flag, int idx)
{
	assert(idx < sizeof(flag) * 8 && idx >= 0);
	unsigned mask = 1 << idx;
	bool val = (flag & mask) != 0;
	return val;
}

tExpTuple::tExpTuple()
{
	mID = 0;
	mReward = 0;
	mFlags = 0;
}

tExpTuple::tExpTuple(int state_size, int action_size)
{
	mReward = 0;
	mFlags = 0;
	mStateBeg = Eigen::VectorXd::Zero(state_size);
	mAction = Eigen::VectorXd::Zero(action_size);
	mStateEnd = Eigen::VectorXd::Zero(state_size);
}

void tExpTuple::ClearFlags()
{
	mFlags = 0;
}

void tExpTuple::SetFlag(bool val, int idx)
{
	assert(idx < sizeof(mFlags) * 8 && idx >= 0);
	unsigned mask = 1 << idx;
	if (val)
	{
		mFlags |= mask;
	}
	else
	{
		mFlags &= ~mask;
	}
}

bool tExpTuple::GetFlag(int idx) const
{
	return TestFlag(mFlags, idx);
}
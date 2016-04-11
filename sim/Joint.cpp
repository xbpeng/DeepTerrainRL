#include "Joint.h"
#include <iostream>

const double gDefaultTorqueLimit = std::numeric_limits<double>::infinity();

cJoint::cJoint()
{
	mTorqueLimit = gDefaultTorqueLimit;
}

cJoint::~cJoint()
{
	Clear();
}

void cJoint::Init(std::shared_ptr<cWorld>& world, std::shared_ptr<cSimObj> parent, std::shared_ptr<cSimObj> child,
				const cWorld::tJointParams& params)
{
	Clear();

	mParams = params;
	mWorld = world;
	mParent = parent;
	mChild = child;

	mCons = mWorld->AddJoint(mParent.get(), mChild.get(), params);

	tMatrix rot = cMathUtil::RotateMat(mParams.mAnchorAxis1, -mParams.mAnchorTheta1);;
	tMatrix trans = cMathUtil::TranslateMat(mParams.mAnchor1);
	mJointChildTrans = trans * rot;

	assert(IsValid());
}

void cJoint::Clear()
{
	if (IsValid())
	{
		mWorld->RemoveConstraint(mCons);
	}
	mParams = cWorld::tJointParams();
	mCons.Clear();
	mWorld.reset();
	mParent.reset();
	mChild.reset();
	mJointChildTrans.setIdentity();

	ClearTorque();
}

bool cJoint::IsValid() const
{
	return(mWorld != nullptr) && (mChild != nullptr) && mCons.IsValid();
}

tVector cJoint::CalcAxisWorld() const
{
	tVector axis_rel = GetAxisRel();
	axis_rel[3] = 0;
	tMatrix trans = BuildWorldTrans();
	tVector axis = trans * axis_rel;
	return axis;
}

const tVector& cJoint::GetAxisRel() const
{
	assert(mParams.mType == cWorld::eJointTypeHinge ||
			// TODO Might need to change this method to return prismatic axis instead
			(mParams.mType == cWorld::eJointTypePrismatic));
	return mParams.mAxis;
}

bool cJoint::HasParent() const
{
	return mParent != nullptr;
}

void cJoint::CalcRotation(tVector& out_axis, double& out_theta) const
{
	assert(IsValid());
	switch (mParams.mType)
	{
	case cWorld::eJointTypeHinge:
		CalcRotationHinge(out_axis, out_theta);
		break;
	case cWorld::eJointTypePrismatic:
		out_axis = tVector(0, 0, 1, 0);
		out_theta = CalcDisplacementPrismatic();
		break;
	default:
		printf("Unsupported constraint type for cJoint::CalcRotation()\n");
		break;
	}
}

void cJoint::CalcWorldRotation(tVector& out_axis, double& out_theta) const
{
	tMatrix mat = BuildWorldTrans();
	cMathUtil::RotMatToAxisAngle(mat, out_axis, out_theta);
}

void cJoint::GetChildRotation(tVector& out_axis, double& out_theta) const
{
	mChild->GetRotation(out_axis, out_theta);
}

tMatrix cJoint::BuildWorldTrans() const
{
	tMatrix mat = mChild->GetWorldTransform();
	double theta = mParams.mRefTheta;
	tVector anchor = GetChildAnchor();
	mat = mat * mJointChildTrans;
	return mat;
}

tVector cJoint::CalcJointVelRel() const
{
	tVector joint_vel = CalcJointVel();
	joint_vel[3] = 0;

	tMatrix trans = BuildWorldTrans();
	joint_vel = trans.transpose() * joint_vel;
	return joint_vel;
}

tVector cJoint::CalcJointVel() const
{
	tVector ang_velp = tVector::Zero();
	if (HasParent())
	{
		ang_velp = mParent->GetAngularVelocity();
	}
	
	tVector ang_velc = mChild->GetAngularVelocity();

	tVector joint_vel = ang_velc - ang_velp;
	return joint_vel;
}

void cJoint::AddTorque(const tVector& torque)
{
	mTotalTorque += torque;
}

const tVector& cJoint::GetTorque() const
{
	return mTotalTorque;
}

void cJoint::ApplyForce()
{
	tVector force = mTotalTorque;

	ClampTotalTorque(force);
	mTotalTorque = force;
	force[0] = 0;
	force[1] = force[2];
	force[2] = 0.0;
	force[3] = 0;

	tMatrix trans = BuildWorldTrans();
	force = trans * force;

	if (HasParent())
	{
		mParent->ApplyForce(-force);
	}
	mChild->ApplyForce(force);
}

void cJoint::ApplyTorque()
{
	tVector torque = mTotalTorque;

	switch (mParams.mType)
	{
	case cWorld::eJointTypeHinge:
		// for hinge joints make sure toque is along axis
		torque = mParams.mAxis.dot(torque) * mParams.mAxis;
		break;
	case cWorld::eJointTypePrismatic:
		this->ApplyForce();
		return;
		break;
	default:
		break;
	}

	ClampTotalTorque(torque);
	mTotalTorque = torque;

	tMatrix trans = BuildWorldTrans();
	torque[3] = 0;
	torque = trans * torque;

	if (HasParent())
	{
		mParent->ApplyTorque(-torque);
	}
	mChild->ApplyTorque(torque);
}

void cJoint::ClearTorque()
{
	mTotalTorque = tVector::Zero();
}

tVector cJoint::GetPos() const
{
	const tVector anchor = mParams.mAnchor1;
	tVector world_pos = mChild->LocalToWorldPos(anchor);
	return world_pos;
}

tVector cJoint::GetVel() const
{
	const tVector anchor = mParams.mAnchor1;
	tVector world_vel = mChild->GetLinearVelocity(anchor);
	return world_vel;
}

void cJoint::SetTorqueLimit(double lim)
{
	mTorqueLimit = lim;
}

const tVector& cJoint::GetParentAnchor() const
{
	return mParams.mAnchor0;
}

const tVector& cJoint::GetChildAnchor() const
{
	return mParams.mAnchor1;
}

const std::shared_ptr<cSimObj>& cJoint::GetParent() const
{
	return mParent;
}

const std::shared_ptr<cSimObj>& cJoint::GetChild() const
{
	return mChild;
}

void cJoint::CalcRotationHinge(tVector& out_axis, double& out_theta) const
{
	mWorld->CalcRotationHinge(this, out_axis, out_theta);
}

double cJoint::CalcDisplacementPrismatic() const
{
	return mWorld->CalcDisplacementPrismatic(this);
}

void cJoint::ClampTotalTorque(tVector& out_torque) const
{
	double mag = out_torque.norm();
	if (mag > mTorqueLimit)
	{
		out_torque *= mTorqueLimit / mag;
	}
}

const cWorld::tConstraintHandle& cJoint::GetConstraintHandle() const
{
	return mCons;
}

double cJoint::GetRefTheta() const
{
	return mParams.mRefTheta;
}
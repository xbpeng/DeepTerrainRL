#pragma once

#include "SimObj.h"

// for now joints are assumed to be hinge joints fixed along the z axis
class cJoint
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cJoint();
	virtual ~cJoint();

	virtual void Init(std::shared_ptr<cWorld>& world, std::shared_ptr<cSimObj> parent, std::shared_ptr<cSimObj> child,
						const cWorld::tJointParams& params);
	virtual void Clear();
	virtual bool IsValid() const;

	virtual tVector CalcAxisWorld() const;
	virtual const tVector& GetAxisRel() const;
	virtual bool HasParent() const;

	virtual void CalcRotation(tVector& out_axis, double& out_theta) const;
	virtual void CalcWorldRotation(tVector& out_axis, double& out_theta) const;
	virtual void GetChildRotation(tVector& out_axis, double& out_theta) const;
	virtual tMatrix BuildWorldTrans() const;

	// joint velocity in local coordinates
	virtual tVector CalcJointVelRel() const;
	virtual tVector CalcJointVel() const;

	virtual void AddTorque(const tVector& torque);
	virtual const tVector& GetTorque() const;
	virtual void ApplyTorque();
	virtual void ApplyForce();
	virtual void ClearTorque();
	virtual tVector GetPos() const;
	virtual tVector GetVel() const;
	virtual void SetTorqueLimit(double lim);

	virtual const tVector& GetParentAnchor() const;
	virtual const tVector& GetChildAnchor() const;
	virtual const std::shared_ptr<cSimObj>& GetParent() const;
	virtual const std::shared_ptr<cSimObj>& GetChild() const;
	
	virtual void CalcRotationHinge(tVector& out_axis, double& out_theta) const;
	virtual double CalcDisplacementPrismatic() const;
	virtual void ClampTotalTorque(tVector& out_torque) const;

	virtual const cWorld::tConstraintHandle& GetConstraintHandle() const;
	virtual double GetRefTheta() const;

protected:
	cWorld::tJointParams mParams;
	std::shared_ptr<cWorld> mWorld;
	std::shared_ptr<cSimObj> mParent;
	std::shared_ptr<cSimObj> mChild;
	cWorld::tConstraintHandle mCons;
	tMatrix mJointChildTrans;

	// torques are stored in local coordinates
	tVector mTotalTorque;
	double mTorqueLimit;

};

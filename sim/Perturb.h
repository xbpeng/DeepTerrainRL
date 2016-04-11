#pragma once

#include <memory>
#include "util/MathUtil.h"

class cSimObj;

struct tPerturb
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum ePerturb
	{
		ePerturbForce,
		ePerturbTorque,
		ePerturbMax,
		ePerturbInvalid
	};

	static tPerturb BuildForce();

	ePerturb mType;
	cSimObj* mObj;
	tVector mPosRel;
	tVector mPerturb;
	double mDuration;
	double mTime;

	tPerturb();
	tPerturb(ePerturb type, cSimObj* obj, const tVector& pos_rel, 
			const tVector& perturb, double duration);

	virtual ~tPerturb();

	virtual bool HasExpired() const;
	virtual void Clear();
	virtual bool IsValid() const;
	virtual void Update(double time_step);

protected:
	virtual void ApplyForce();
	virtual void ApplyTorque();
};
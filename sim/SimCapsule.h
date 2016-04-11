#pragma once

#include "sim/SimObj.h"

class cSimCapsule : public cSimObj
{
public:
	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tParams();

		eType mType;
		double mMass;
		double mFriction;
		tVector mPos;
		double mHeight;
		double mRadius;
		tVector mAxis;
		double mTheta;
	};

	cSimCapsule();
	virtual ~cSimCapsule();

	virtual void Init(std::shared_ptr<cWorld> world, const tParams& params);
	virtual double GetHeight() const;
	virtual double GetRadius() const;

	virtual eShape GetShape() const;

protected:
};
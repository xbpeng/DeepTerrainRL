#pragma once

#include "sim/SimObj.h"

class cSimPlane : public cSimObj
{
public:
	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		tParams();

		double mMass;
		double mFriction;
		tVector mOrigin;
		tVector mNormal;
	};

	cSimPlane();
	virtual ~cSimPlane();

	virtual void Init(std::shared_ptr<cWorld> world, const tParams& params);
	virtual tVector GetCoeffs() const;
	virtual eShape GetShape() const;

protected:
};
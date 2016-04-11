#pragma once

#include "sim/SimCharacter.h"

class cSimCharSoftFall : public cSimCharacter
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cSimCharSoftFall();
	virtual ~cSimCharSoftFall();

	virtual bool Init(std::shared_ptr<cWorld> world, const tParams& params);
	virtual void Reset();
	virtual void Update(double time_step);
	virtual bool HasFallen() const;

protected:
	double mFallDistCounter;
	bool mFailFallDist;
	tVector mPrevCheckPos;

	double mFallContatCounter;
	double mSumFallContact;

	virtual void ResetFallDistCounter();
	virtual bool FailFallDist() const;
	virtual void UpdateFallDistCheck(double time_step);

	virtual void ResetFallContactCounter();
	virtual bool FailFallContact() const;
	virtual bool CheckFallContact() const;
	virtual void UpdateFallContactCheck(double time_step);

	virtual bool FailFallMisc() const;
};
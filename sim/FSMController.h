#pragma once

#include "Controller.h"

class cFSMController : public cController
{
public:
	virtual ~cFSMController();

	virtual int GetState() const;
	virtual double GetPhase() const;
	virtual void SetPhase(double phase);

protected:
	int mState;
	double mPhase;

	cFSMController();
};
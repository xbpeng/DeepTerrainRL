#include "FSMController.h"

cFSMController::cFSMController()
{
	mState = 0;
	mPhase = 0;
}

cFSMController::~cFSMController()
{
}

int cFSMController::GetState() const
{
	return mState;
}

double cFSMController::GetPhase() const
{
	return mPhase;
}

void cFSMController::SetPhase(double phase)
{
	mPhase = phase;
}
#pragma once

#include "sim/DogControllerMACE.h"

class cGoatControllerMACE : public cDogControllerMACE
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cGoatControllerMACE();
	virtual ~cGoatControllerMACE();

	virtual tVector GetTargetVel() const;

protected:
};
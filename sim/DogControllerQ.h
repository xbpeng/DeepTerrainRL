#pragma once

#include "sim/DogController.h"
#include "sim/BaseControllerQ.h"

class cDogControllerQ : public virtual cDogController, public virtual cBaseControllerQ
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDogControllerQ();
	virtual ~cDogControllerQ();

protected:

};
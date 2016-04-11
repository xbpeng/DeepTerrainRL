#pragma once

#include "sim/RaptorController.h"
#include "sim/BaseControllerQ.h"

class cRaptorControllerQ : public virtual cRaptorController, public virtual cBaseControllerQ
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cRaptorControllerQ();
	virtual ~cRaptorControllerQ();

protected:

};
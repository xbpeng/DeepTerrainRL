#pragma once

#include "sim/BaseControllerCacla.h"
#include "sim/RaptorController.h"

class cRaptorControllerCacla : public virtual cRaptorController, public virtual cBaseControllerCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cRaptorControllerCacla();
	virtual ~cRaptorControllerCacla();
	
protected:
	virtual void UpdateAction();
	virtual bool IsCurrActionCyclic() const;
};
#pragma once

#include "sim/BaseControllerCacla.h"
#include "sim/DogController.h"

class cDogControllerCacla : public virtual cDogController, public virtual cBaseControllerCacla
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cDogControllerCacla();
	virtual ~cDogControllerCacla();
	
protected:
	virtual void UpdateAction();
	virtual bool IsCurrActionCyclic() const;
};
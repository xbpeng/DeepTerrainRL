#pragma once

#include "sim/RaptorController.h"
#include "sim/BaseControllerMACE.h"

class cRaptorControllerMACE : public virtual cRaptorController, public virtual cBaseControllerMACE
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	cRaptorControllerMACE();
	virtual ~cRaptorControllerMACE();

	virtual void Reset();

protected:
	
	virtual bool IsCurrActionCyclic() const;

	virtual void UpdateAction();
	virtual void BuildBaseAction(int action_id, tAction& out_action) const;
	virtual void ProcessCommand(tAction& out_action);
	virtual int AssignFragID(int a_id) const;

	virtual void BuildActorBias(int a_id, Eigen::VectorXd& out_bias) const;
};
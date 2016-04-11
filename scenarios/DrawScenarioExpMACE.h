#pragma once
#include <memory>

#include "DrawScenarioExp.h"

class cDrawScenarioExpMACE : public cDrawScenarioExp
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioExpMACE(cCamera& cam);
	virtual ~cDrawScenarioExpMACE();

protected:
	virtual void BuildScene();
};
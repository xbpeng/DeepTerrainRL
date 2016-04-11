#pragma once
#include <memory>

#include "DrawScenarioExp.h"

class cDrawScenarioExpCacla : public cDrawScenarioExp
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioExpCacla(cCamera& cam);
	virtual ~cDrawScenarioExpCacla();

protected:
	virtual void BuildScene();
};
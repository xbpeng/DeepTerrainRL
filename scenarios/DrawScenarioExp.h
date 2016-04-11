#pragma once
#include <memory>

#include "DrawScenarioSimChar.h"

class cDrawScenarioExp : public cDrawScenarioSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioExp(cCamera& cam);
	virtual ~cDrawScenarioExp();

protected:
	virtual void BuildScene();
	virtual void ResetCallback();
};
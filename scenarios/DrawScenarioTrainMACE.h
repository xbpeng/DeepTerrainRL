#pragma once
#include <memory>

#include "DrawScenarioTrain.h"

class cDrawScenarioTrainMACE: public cDrawScenarioTrain
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioTrainMACE(cCamera& cam);
	virtual ~cDrawScenarioTrainMACE();

protected:

	virtual void BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const;
};
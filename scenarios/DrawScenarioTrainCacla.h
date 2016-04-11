#pragma once
#include <memory>

#include "DrawScenarioTrain.h"

class cDrawScenarioTrainCacla: public cDrawScenarioTrain
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioTrainCacla(cCamera& cam);
	virtual ~cDrawScenarioTrainCacla();

protected:

	virtual void BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const;
};
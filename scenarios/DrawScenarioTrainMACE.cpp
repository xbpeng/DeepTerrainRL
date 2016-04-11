#include "DrawScenarioTrainMACE.h"
#include "scenarios/ScenarioTrainMACE.h"

cDrawScenarioTrainMACE::cDrawScenarioTrainMACE(cCamera& cam)
	: cDrawScenarioTrain(cam)
{
}

cDrawScenarioTrainMACE::~cDrawScenarioTrainMACE()
{
}

void cDrawScenarioTrainMACE::BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioTrainMACE>(new cScenarioTrainMACE());
}
#include "DrawScenarioExpCacla.h"
#include "scenarios/ScenarioExpCacla.h"

cDrawScenarioExpCacla::cDrawScenarioExpCacla(cCamera& cam)
	: cDrawScenarioExp(cam)
{
}

cDrawScenarioExpCacla::~cDrawScenarioExpCacla()
{
}

void cDrawScenarioExpCacla::BuildScene()
{
	mScene = std::shared_ptr<cScenarioSimChar>(new cScenarioExpCacla());
	tCallbackFunc func = std::bind(&cDrawScenarioExpCacla::ResetCallback, this);
	mScene->SetResetCallback(func);
}
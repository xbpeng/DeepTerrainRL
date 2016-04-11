#include "DrawScenarioExp.h"
#include "scenarios/ScenarioExp.h"

cDrawScenarioExp::cDrawScenarioExp(cCamera& cam)
	: cDrawScenarioSimChar(cam)
{
}

cDrawScenarioExp::~cDrawScenarioExp()
{
}

void cDrawScenarioExp::BuildScene()
{
	mScene = std::shared_ptr<cScenarioSimChar>(new cScenarioExp());
	tCallbackFunc func = std::bind(&cDrawScenarioExp::ResetCallback, this);
	mScene->SetResetCallback(func);
}

void cDrawScenarioExp::ResetCallback()
{
	ResetCamera();
	mTracer.Reset();
}
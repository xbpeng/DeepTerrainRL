#include "DrawScenarioTrain.h"

cDrawScenarioTrain::cDrawScenarioTrain(cCamera& cam)
	: cDrawScenarioSimChar(cam)
{
}

cDrawScenarioTrain::~cDrawScenarioTrain()
{
}

void cDrawScenarioTrain::Init()
{
	cDrawScenarioSimInteractive::Init();

	BuildTrainScene(mTrain);

	mTrain->SetExpPoolSize(1);
	mTrain->ParseArgs(mArgParser);
	mTrain->Init();
	
	mScene = mTrain->GetExpScene(0);
	tCallbackFunc func = std::bind(&cDrawScenarioTrain::ResetCallback, this);
	mScene->SetResetCallback(func);

	mEnableTrace = false;
	InitTracer();
}

void cDrawScenarioTrain::Reset()
{
	cDrawScenarioSimInteractive::Reset();
	mTrain->Reset();
}

void cDrawScenarioTrain::Clear()
{
	cDrawScenarioSimInteractive::Clear();
	mTrain->Clear();
}

void cDrawScenarioTrain::Update(double time_elapsed)
{
	cDrawScenarioSimInteractive::Update(time_elapsed);
	mTrain->Update(time_elapsed);

	if (mEnableTrace)
	{
		UpdateTracer(time_elapsed);
	}

	UpdateCamera();
}

void cDrawScenarioTrain::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioSimChar::Keyboard(key, x, y);

	switch (key)
	{
	case 't':
		ToggleTraining();
		break;
	default:
		break;
	}
}

void cDrawScenarioTrain::Shutdown()
{
	mTrain->Shutdown();
}

void cDrawScenarioTrain::BuildTrainScene(std::shared_ptr<cScenarioTrain>& out_scene) const
{
	out_scene = std::shared_ptr<cScenarioTrain>(new cScenarioTrain());
}

void cDrawScenarioTrain::ToggleTraining()
{
	mTrain->ToggleTraining();

	bool training = mTrain->TrainingEnabled();
	if (training)
	{
		printf("Training enabled\n");
	}
	else
	{
		printf("Training disabled\n");
	}
}

std::string cDrawScenarioTrain::GetName() const
{
	return mTrain->GetName();
}

void cDrawScenarioTrain::ResetCallback()
{
	mTracer.Reset();
}
#include "DrawScenarioPoliEval.h"
#include "scenarios/ScenarioPoliEval.h"

cDrawScenarioPoliEval::cDrawScenarioPoliEval(cCamera& cam)
	: cDrawScenarioSimChar(cam)
{
	mDrawPoliInfo = true;
	mRandSeed = 0;
}

cDrawScenarioPoliEval::~cDrawScenarioPoliEval()
{
}

void cDrawScenarioPoliEval::ParseArgs(const cArgParser& parser)
{
	cDrawScenarioSimChar::ParseArgs(parser);

	int rand_seed = 0;
	parser.ParseInt("poli_eval_rand_seed", rand_seed);
	mRandSeed = static_cast<unsigned long>(rand_seed);
}

void cDrawScenarioPoliEval::Init()
{
	cDrawScenarioSimChar::Init();

	std::shared_ptr<cScenarioPoliEval> eval_scene = std::static_pointer_cast<cScenarioPoliEval>(mScene);
	bool valid_seed = mRandSeed != 0;
	if (valid_seed)
	{
		eval_scene->SetRandSeed(mRandSeed);
		eval_scene->Reset(); // rebuild ground
	}
}

void cDrawScenarioPoliEval::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioSimChar::Keyboard(key, x, y);

	if (key >= '1' && key <= '9')
	{
		CommandAction(key - '1');
	}
}

std::string cDrawScenarioPoliEval::BuildTextInfoStr() const
{
	const auto& character = mScene->GetCharacter();
	tVector com = character->CalcCOM();
	tVector com_vel = character->CalcCOMVel();
	char buffer[128];

	auto poli_eval = std::static_pointer_cast<cScenarioPoliEval>(mScene);
	double avg_dist = poli_eval->GetAvgDist();
	int num_samples = poli_eval->GetNumEpisodes();
	sprintf(buffer, "Avg Dist: %.3f (%i)\n", avg_dist, num_samples);

	std::string eval_str(buffer);
	std::string info_str = cDrawScenarioSimChar::BuildTextInfoStr();
	info_str += eval_str;

	return info_str;
}

void cDrawScenarioPoliEval::BuildScene()
{
	std::shared_ptr<cScenarioPoliEval> eval_scene = std::shared_ptr<cScenarioPoliEval>(new cScenarioPoliEval());
	tCallbackFunc func = std::bind(&cDrawScenarioPoliEval::ResetCallback, this);
	eval_scene->SetResetCallback(func);
	mScene = eval_scene;
}

void cDrawScenarioPoliEval::ResetCallback()
{
	ResetCamera();
	mTracer.Reset();
}

void cDrawScenarioPoliEval::CommandAction(int a)
{
	std::shared_ptr<cScenarioSimChar> sim_scene = std::static_pointer_cast<cScenarioSimChar>(mScene);
	const auto& character = sim_scene->GetCharacter();
	const std::shared_ptr<cCharController>& ctrl = character->GetController();
	ctrl->CommandAction(a);
}
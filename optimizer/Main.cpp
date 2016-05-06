#include <memory>
#include <signal.h>

#include "scenarios/ScenarioTrain.h"
#include "scenarios/ScenarioTrainCacla.h"
#include "scenarios/ScenarioTrainMACE.h"
#include "scenarios/OptScenarioPoliEval.h"
#include "util/ArgParser.h"

// arg parser
cArgParser gArgParser;
std::shared_ptr<cScenario> gScenario = nullptr;
int gArgc = 0;
char** gArgv = nullptr;
int gNumThreads = 1;

const double gTimeStep = 1.0 / 30;

void ParseArgs(int argc, char** argv)
{
	gArgParser = cArgParser(argv, argc);

	std::string arg_file = "";
	gArgParser.ParseString("arg_file", arg_file);
	if (arg_file != "")
	{
		// append the args from the file to the ones from the commandline
		// this allows the cmd args to overwrite the file args
		gArgParser.AppendArgs(arg_file);
	}
	gArgParser.ParseInt("num_threads", gNumThreads);
}

void SetupScenario()
{
	std::string scenario_name = "";
	gArgParser.ParseString("scenario", scenario_name);

	if (scenario_name == "train")
	{
		std::shared_ptr<cScenarioTrain> train = std::shared_ptr<cScenarioTrain>(new cScenarioTrain());
		train->SetTimeStep(gTimeStep);
		train->SetExpPoolSize(gNumThreads);

		gScenario = std::shared_ptr<cScenario>(train);
	}
	else if (scenario_name == "train_cacla")
	{
		std::shared_ptr<cScenarioTrainCacla> train = std::shared_ptr<cScenarioTrainCacla>(new cScenarioTrainCacla());
		train->SetTimeStep(gTimeStep);
		train->SetExpPoolSize(gNumThreads);

		gScenario = std::shared_ptr<cScenario>(train);
	}
	else if (scenario_name == "train_mace")
	{
		std::shared_ptr<cScenarioTrainMACE> train = std::shared_ptr<cScenarioTrainMACE>(new cScenarioTrainMACE());
		train->SetTimeStep(gTimeStep);
		train->SetExpPoolSize(gNumThreads);

		gScenario = std::shared_ptr<cScenario>(train);
	}
	else if (scenario_name == "poli_eval")
	{
		std::shared_ptr<cOptScenarioPoliEval> eval = std::shared_ptr<cOptScenarioPoliEval>(new cOptScenarioPoliEval());
		eval->SetTimeStep(gTimeStep);
		eval->SetPoolSize(gNumThreads);

		gScenario = std::shared_ptr<cScenario>(eval);
	}
	else
	{
		printf("No valid scenario specified\n");
	}

	if (gScenario != NULL)
	{
		gScenario->ParseArgs(gArgParser);
		printf("Loaded scenario: %s\n", gScenario->GetName().c_str());
		gScenario->Init();
	}
}

void RunScene()
{
	if (gScenario != nullptr)
	{
		gScenario->Run();
	}
}

void CleanUp()
{
	gScenario.reset();
	gArgParser.Clear();
}

void SigHandler(int sig)
{
	if (gScenario != nullptr)
	{
		gScenario->Shutdown();
	}
}

int main(int argc, char** argv)
{
	signal(SIGINT, &SigHandler);

	gArgc = argc;
	gArgv = argv;
	ParseArgs(gArgc, gArgv);

	SetupScenario();
	RunScene();

	CleanUp();

	return EXIT_SUCCESS;
}
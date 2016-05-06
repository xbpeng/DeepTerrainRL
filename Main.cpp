#include <iostream>
#include <caffe/caffe.hpp>

#include "util/FileUtil.h"
#include "util/ArgParser.h"
#include "scenarios/DrawScenarioSimChar.h"
#include "scenarios/DrawScenarioExp.h"
#include "scenarios/DrawScenarioExpCacla.h"
#include "scenarios/DrawScenarioExpMACE.h"
#include "scenarios/DrawScenarioTrain.h"
#include "scenarios/DrawScenarioTrainCacla.h"
#include "scenarios/DrawScenarioTrainMACE.h"
#include "scenarios/DrawScenarioPoliEval.h"

#include "render/Camera.h"
#include "render/DrawUtil.h"
#include "render/TextureDesc.h"

// Dimensions of the window we are drawing into.
int gWinWidth = 800;
int gWinHeight = static_cast<int>(gWinWidth * 9.0 / 16.0);
bool gReshaping = false;

const tVector gBKGColor = tVector(0.97, 0.97, 1, 0);
//const tVector gBKGColor = tVector(1, 1, 1, 0);

// camera attributes
double gViewWidth = 4.5;
//double gViewWidth = 6.5;
double gViewHeight = (gViewWidth * gWinHeight) / gWinWidth;
double gViewNearZ = 2;
double gViewFarZ = 50;

// intermediate frame buffers
std::unique_ptr<cTextureDesc> gDefaultFrameBuffer;
std::shared_ptr<cTextureDesc> gIntermediateFrameBuffer;

tVector gCameraPosition = tVector(0, 0, 30, 0);
tVector gCameraFocus = tVector(gCameraPosition[0], gCameraPosition[1], 0.0, 0.0);
tVector gCameraUp = tVector(0, 1, 0, 0);

cCamera gCamera;

// anim
const double gFPS = 30.0;
const double gAnimStep = 1.0 / gFPS;
const int gDisplayAnimTime = static_cast<int>(gAnimStep * 1000);
bool gAnimate = true;

int gPrevTime = 0;
int gNextTime = 0;
int gDispalyPrevTime = 0;
double gUpdatesPerSec = 0;

double gPlaybackSpeed = 1.0;
const double gPlaybackDelta = 0.05;

bool gForceClear = false;
bool gRenderFilmStrip = false;
const double gFilmStripPeriod = 0.5;
tVector gPrevCamPos = tVector::Zero();

// arg parser
cArgParser gArgParser;
std::shared_ptr<cDrawScenario> gScenario = NULL;
int gArgc = 0;
char** gArgv = NULL;


void SetupCamProjection()
{
	gCamera.SetupGLProj();
}

void ResizeCamera()
{
	gViewWidth = (gViewHeight * gWinWidth) / gWinHeight;
	gCamera.Resize(gViewWidth, gViewHeight);
	gForceClear = true;
}

void InitCamera()
{
	gCamera = cCamera(gCameraPosition, gCameraFocus, gCameraUp,
						gViewWidth, gViewHeight, gViewNearZ, gViewFarZ);
	gCamera.SetProj(cCamera::eProjOrtho);
	ResizeCamera();
}

void ClearScenario()
{
	gScenario = NULL;
}

void SetupScenario()
{
	InitCamera();
	ClearScenario();

	std::string scenario_name = "";
	gArgParser.ParseString("scenario", scenario_name);

	if (scenario_name == "sim_char")
	{
		gScenario = std::shared_ptr<cDrawScenarioSimChar>(new cDrawScenarioSimChar(gCamera));
	}
	else if (scenario_name == "exp")
	{
		gScenario = std::shared_ptr<cDrawScenarioExp>(new cDrawScenarioExp(gCamera));
	}
	else if (scenario_name == "exp_cacla")
	{
		gScenario = std::shared_ptr<cDrawScenarioExpCacla>(new cDrawScenarioExpCacla(gCamera));
	}
	else if (scenario_name == "exp_mace")
	{
		gScenario = std::shared_ptr<cDrawScenarioExpMACE>(new cDrawScenarioExpMACE(gCamera));
	}
	else if (scenario_name == "train")
	{
		gScenario = std::shared_ptr<cDrawScenarioTrain>(new cDrawScenarioTrain(gCamera));
	}
	else if (scenario_name == "train_cacla")
	{
		gScenario = std::shared_ptr<cDrawScenarioTrainCacla>(new cDrawScenarioTrainCacla(gCamera));
	}
	else if (scenario_name == "train_mace")
	{
		gScenario = std::shared_ptr<cDrawScenarioTrainMACE>(new cDrawScenarioTrainMACE(gCamera));
	}
	else if (scenario_name == "poli_eval")
	{
		gScenario = std::shared_ptr<cDrawScenarioPoliEval>(new cDrawScenarioPoliEval(gCamera));
	}

	if (gScenario != NULL)
	{
		gScenario->ParseArgs(gArgParser);
		gScenario->Init();
		printf("Loaded scenario: %s\n", gScenario->GetName().c_str());
	}
}

void UpdateScenario(double time_step)
{
	if (gScenario != NULL)
	{
		int num_steps = 1;
		if (gRenderFilmStrip)
		{
			num_steps = static_cast<int>(gFilmStripPeriod / time_step);
		}

		for (int i = 0; i < num_steps; ++i)
		{
			gScenario->Update(time_step);
		}
	}
}

void DrawInfo()
{
	if (!gRenderFilmStrip)
	{
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		const double aspect = gCamera.GetAspectRatio();
		const double text_size = 0.09;
		const tVector scale = tVector(text_size / aspect, text_size, 1, 0);
		const double line_offset = text_size;

		cDrawUtil::SetLineWidth(1.5);
		cDrawUtil::SetColor(tVector(0, 0, 0, 0.5));

		cDrawUtil::Translate(tVector(-0.96, 0.88, -1, 0));
		double curr_fps = gUpdatesPerSec;

		char buffer[128];
		sprintf(buffer, "FPS: %.2f\nPlayback Speed: %.2fx\n", curr_fps, gPlaybackSpeed);

		std::string text_info = std::string(buffer);
		if (gScenario != nullptr)
		{
			text_info += gScenario->BuildTextInfoStr();
		}

		cDrawUtil::DrawString(text_info.c_str(), scale);

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();

		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	}
}

void ClearFrame()
{
	bool clear = true;

	if (gRenderFilmStrip && !gForceClear)
	{
		const tVector& cam_pos = gCamera.GetPosition();
		if (cam_pos != gPrevCamPos)
		{
			gPrevCamPos = cam_pos;
		}
		else
		{
			clear = false;
		}
	}

	if (clear)
	{
		cDrawUtil::ClearColor(gBKGColor);
		cDrawUtil::ClearDepth(1);
	}

	gForceClear = false;
}

void DrawScene()
{
	if (gScenario != NULL)
	{
		gScenario->Draw();
	}
}

void CopyFrame()
{
	cDrawUtil::CopyTexture(*gIntermediateFrameBuffer, *gDefaultFrameBuffer);
}

void UpdateIntermediateBuffer()
{
	if (!gReshaping)
	{
		if (gWinWidth != gIntermediateFrameBuffer->GetWidth()
			|| gWinHeight != gIntermediateFrameBuffer->GetHeight())
		{
			gIntermediateFrameBuffer->Reshape(gWinWidth, gWinHeight);
			gScenario->Reshape(gWinWidth, gWinHeight);
		}
	}
}

void Display(void)
{
	UpdateIntermediateBuffer();

	glMatrixMode(GL_PROJECTION);
	SetupCamProjection();

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gIntermediateFrameBuffer->BindBuffer();
	ClearFrame();
	DrawScene();
	DrawInfo();
	gIntermediateFrameBuffer->UnbindBuffer();

	CopyFrame();

	glutSwapBuffers();
	gDispalyPrevTime = glutGet(GLUT_ELAPSED_TIME);

	gReshaping = false;
}

void Reshape(int w, int h)
{
	gReshaping = true;

	gWinWidth = w;
	gWinHeight = h;

	glViewport(0, 0, gWinWidth, gWinHeight);
	gDefaultFrameBuffer->Reshape(w, h);

	UpdateScenario(0);
	ResizeCamera();

	glMatrixMode(GL_PROJECTION);
	SetupCamProjection();

	glutPostRedisplay();
}

void StepAnim(double time_step)
{
	UpdateScenario(time_step);
	gAnimate = false;
	glutPostRedisplay();
}

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
}

void InitTime()
{
	gPrevTime = glutGet(GLUT_ELAPSED_TIME);
	gNextTime = gPrevTime;
	gDispalyPrevTime = gPrevTime;
	gUpdatesPerSec = 0.f;
}

void Reload()
{
	ParseArgs(gArgc, gArgv);
	SetupScenario();
	InitTime();
	gForceClear = true;
}

void Reset()
{
	if (gScenario != NULL)
	{
		gScenario->Reset();
	}
	gForceClear = true;
}

void Update(double time_elapsed)
{
	UpdateScenario(time_elapsed);
}

int GetNumTimeSteps()
{
	int num_steps = static_cast<int>(gPlaybackSpeed);
	if (num_steps == 0)
	{
		num_steps = 1;
	}
	num_steps = std::abs(num_steps);
	return num_steps;
}

int CalcDisplayAnimTime()
{
	int anim_time = static_cast<int>(gDisplayAnimTime * GetNumTimeSteps() / gPlaybackSpeed);
	anim_time = std::abs(anim_time);
	return anim_time;
}

void Shutdown()
{
	if (gScenario != nullptr)
	{
		gScenario->Shutdown();
	}
	exit(0);
}

void Animate(int callback_val)
{
	if (gAnimate)
	{
		int num_steps = GetNumTimeSteps();
		int timer_step = CalcDisplayAnimTime();
		
		glutTimerFunc(timer_step, Animate, 0);

		int current_time = glutGet(GLUT_ELAPSED_TIME);
		int elapsedTime = current_time - gPrevTime;
		gPrevTime = current_time;

		double timestep = (gPlaybackSpeed < 0) ? -gAnimStep : gAnimStep;
		for (int i = 0; i < num_steps; ++i)
		{
			Update(timestep);
		}
		
		glutPostRedisplay();
		gUpdatesPerSec = num_steps / (elapsedTime * 0.001);
	}

	if (gScenario != nullptr)
	{
		if (gScenario->IsDone())
		{
			Shutdown();
		}
	}
}

void ToggleAnimate()
{
	gAnimate = !gAnimate;
	if (gAnimate)
	{
		glutTimerFunc(gDisplayAnimTime, Animate, 0);
	}
}

void ChangePlaybackSpeed(double delta)
{
	double prev_playback = gPlaybackSpeed;
	gPlaybackSpeed += delta;

	if (std::abs(prev_playback) < 0.0001 && std::abs(gPlaybackSpeed) > 0.0001)
	{
		glutTimerFunc(gDisplayAnimTime, Animate, 0);
	}
}

void ToogleFilmStrip()
{
	gRenderFilmStrip = !gRenderFilmStrip;
	gForceClear = true;

	if (gRenderFilmStrip)
	{
		printf("Filmstrip mode enabled\n");
	}
	else
	{
		printf("Filmstrip mode disabled\n");
	}
}

void Keyboard(unsigned char key, int x, int y) {

	if (gScenario != NULL)
	{
		gScenario->Keyboard(key, x, y);
	}

	bool update = false;
	switch (key) {
		// Quit.
#ifndef _LINUX_
	case CTRL_CLOSE_EVENT:
	case CTRL_C_EVENT:
#endif
	case 27: // escape
		Shutdown();
		break;
	case 13: // enter
		break;
	case ' ':
		ToggleAnimate();
		break;
	case '>':
		StepAnim(gAnimStep);
		break;
	case '<':
		StepAnim(-gAnimStep);
		break;
	case ',':
		ChangePlaybackSpeed(-gPlaybackDelta);
		break;
	case '.':
		ChangePlaybackSpeed(gPlaybackDelta);
		break;
	case '/':
		ChangePlaybackSpeed(-gPlaybackSpeed + 1);
		break;
	case 'l':
		Reload();
		break;
	case 'q':
		ToogleFilmStrip();
		break;
	case 'r':
		Reset();
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

void MouseClick(int button, int state, int x, int y)
{
	double screen_x = static_cast<double>(x) / gWinWidth;
	double screen_y = static_cast<double>(y) / gWinHeight;
	screen_x = (screen_x - 0.5f) * 2.f;
	screen_y = (screen_y - 0.5f) * -2.f;

	if (gScenario != NULL)
	{
		gScenario->MouseClick(button, state, screen_x, screen_y);
	}
	glutPostRedisplay();
}


void MouseMove(int x, int y)
{
	double screen_x = static_cast<double>(x) / gWinWidth;
	double screen_y = static_cast<double>(y) / gWinHeight;
	screen_x = (screen_x - 0.5f) * 2.f;
	screen_y = (screen_y - 0.5f) * -2.f;

	if (gScenario != NULL)
	{
		gScenario->MouseMove(screen_x, screen_y);
	}
	glutPostRedisplay();
}

void InitOpenGl(void)
{
	cDrawUtil::InitDrawUtil();
	glewInit();

	gDefaultFrameBuffer = std::unique_ptr<cTextureDesc>(new cTextureDesc(0, 0, 0, gWinWidth, gWinHeight, 1, GL_RGBA, GL_RGBA));
	gIntermediateFrameBuffer = std::shared_ptr<cTextureDesc>(new cTextureDesc(gWinWidth, gWinHeight, GL_RGBA, GL_RGBA, GL_UNSIGNED_BYTE, false));
}

void InitCaffe()
{
	FLAGS_alsologtostderr = 1;
	int caffe_argc = 1; // hack
	caffe::GlobalInit(&caffe_argc, &gArgv);
}

int main(int argc, char** argv)
{
	gArgc = argc;
	gArgv = argv;
	ParseArgs(gArgc, gArgv);

	InitCaffe();

	glutInit(&gArgc, gArgv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(gWinWidth, gWinHeight);
	glutCreateWindow("Terrain RL");

	InitOpenGl();
	SetupScenario();

	Reshape(gWinWidth, gWinHeight);
	glutDisplayFunc(Display);
	glutReshapeFunc(Reshape);
	glutKeyboardFunc(Keyboard);
	glutMouseFunc(MouseClick);
	glutMotionFunc(MouseMove);
	glutTimerFunc(gDisplayAnimTime, Animate, 0);

	InitTime();
	glutMainLoop();

	return EXIT_SUCCESS;
}


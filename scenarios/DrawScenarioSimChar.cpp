#include "DrawScenarioSimChar.h"
#include "sim/BaseControllerMACE.h"
#include "render/DrawUtil.h"
#include "render/DrawObj.h"
#include "render/DrawWorld.h"
#include "render/DrawSimCharacter.h"
#include "render/DrawPerturb.h"
#include "render/DrawGround.h"

const double gLinkWidth = 0.05f;
const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFillTint = tVector(1, 1, 1, 1);
const tVector gVisOffset = tVector(0, 0, 0.5, 0); // offset for visualization elements
const int gTracerBufferSize = 200;
const double gTracerSamplePeriod = 0.033;

const std::string gOutputCharFile = "output/char_state.txt";

cDrawScenarioSimChar::cDrawScenarioSimChar(cCamera& cam)
	: cDrawScenarioSimInteractive(cam)
{
	mDrawInfo = false;
	mDrawPoliInfo = false;
	mEnableTrace = false;
	mDrawFeatures = false;
	mDrawPolicyPlots = false;

	mCamDelta = cam.GetPosition() - cam.GetFocus();
}

cDrawScenarioSimChar::~cDrawScenarioSimChar()
{
}

void cDrawScenarioSimChar::Init()
{
	cDrawScenarioSimInteractive::Init();
	BuildScene();
	mScene->ParseArgs(mArgParser);
	mScene->Init();
	
	mEnableTrace = false;
	InitTracer();
}

void cDrawScenarioSimChar::ParseArgs(const cArgParser& parser)
{
	cDrawScenarioSimInteractive::ParseArgs(parser);
	mArgParser = parser;
}

void cDrawScenarioSimChar::Reset()
{
	mScene->Reset();
	mTracer.Reset();
	cDrawScenarioSimInteractive::Reset();
}

void cDrawScenarioSimChar::Clear()
{
	cDrawScenarioSimInteractive::Clear();
	mScene->Clear();
	mTracer.Clear();
}

void cDrawScenarioSimChar::Update(double time_elapsed)
{
	cDrawScenarioSimInteractive::Update(time_elapsed);
	mScene->Update(time_elapsed);

	if (mEnableTrace)
	{
		UpdateTracer(time_elapsed);
	}
	
	UpdateCamera();
}

void cDrawScenarioSimChar::UpdateTracer(double time_elapsed)
{
	auto ctrl = mScene->GetCharacter()->GetController();

	const std::shared_ptr<cBaseControllerMACE> mace_ctrl = std::dynamic_pointer_cast<cBaseControllerMACE>(ctrl);
	bool is_mace_ctrl = (mace_ctrl != nullptr);
	if (is_mace_ctrl)
	{
		int action_id = ctrl->GetCurrActionID();
		int trace_handle = mTraceHandles[0];
		mTracer.SetTraceColIdx(trace_handle, action_id);
	}

	mTracer.Update(time_elapsed);
}

void cDrawScenarioSimChar::Keyboard(unsigned char key, int x, int y)
{
	cDrawScenarioSimInteractive::Keyboard(key, x, y);

	switch (key)
	{
	case 'c':
		ToggleCamTrackMode(eCamTrackModeStill);
		break;
	case 'f':
		mDrawInfo = !mDrawInfo;
		break;
	case 'g':
		mDrawFeatures = !mDrawFeatures;
		break;
	case 'h':
		mDrawPolicyPlots = !mDrawPolicyPlots;
		break;
	case 'p':
		mDrawPoliInfo = !mDrawPoliInfo;
		break;
	case 's':
		OutputCharState(GetOutputCharFile());
		break;
	case 'x':
		SpawnProjectile();
		break;
	case 'y':
		ToggleTrace();
		break;
	case 'z':
		SpawnBigProjectile();
		break;
	default:
		break;
	}
}

void cDrawScenarioSimChar::MouseClick(int button, int state, double x, double y)
{
	cDrawScenarioSimInteractive::MouseClick(button, state, x, y);
}

void cDrawScenarioSimChar::MouseMove(double x, double y)
{
	cDrawScenarioSimInteractive::MouseMove(x, y);
}

void cDrawScenarioSimChar::Reshape(int w, int h)
{
}

std::string cDrawScenarioSimChar::GetName() const
{
	return mScene->GetName();
}

void cDrawScenarioSimChar::BuildScene()
{
	mScene = std::shared_ptr<cScenarioSimChar>(new cScenarioSimChar());
}

tVector cDrawScenarioSimChar::GetCamTrackPos() const
{
	const auto& character = mScene->GetCharacter();
	return character->CalcCOM();
}

tVector cDrawScenarioSimChar::GetCamStillPos() const
{
	const auto& character = mScene->GetCharacter();
	tVector char_pos = character->CalcCOM();
	
	double cam_w = mCam.GetWidth();
	double cam_h = mCam.GetHeight();
	const auto& ground = mScene->GetGround();

	const int num_samples = 16;
	double ground_samples[num_samples] = { 0 };
	const double pad = std::min(0.5, 0.5 * cam_w);

	double avg_h = 0;

	double min_x = char_pos[0];
	double max_x = char_pos[0] + cam_w;

	int num_valid_samples = 0;
	for (int i = 0; i < num_samples; ++i)
	{
		tVector pos = char_pos;
		pos[0] = static_cast<double>(i) / (num_samples - 1) * (max_x - min_x) + min_x;

		bool valid_sample = true;
		double ground_h = ground->SampleHeight(pos, valid_sample);
		if (valid_sample)
		{
			ground_samples[i] = ground_h;
			avg_h += ground_h;
			++num_valid_samples;
		}
	}
	avg_h /= num_valid_samples;

	std::sort(ground_samples, &(ground_samples[num_samples - 1]));
	double med_h = ground_samples[num_samples / 2];
	double min_h = ground_samples[0];

	tVector track_pos = char_pos;
	double target_h = avg_h;
	//double target_h = min_h;

	//double y_pad = -0.2;
	double y_pad = -0.4;
	track_pos[1] = target_h + y_pad + 0.5 * cam_h;

	return track_pos;
}

void cDrawScenarioSimChar::InitTracer()
{
	mTraceHandles.clear();
	mTracer.Init(gTracerBufferSize, gTracerSamplePeriod);

	tVectorArr tracer_cols;
	tracer_cols.push_back(tVector(0, 0, 1, 0.5));
	tracer_cols.push_back(tVector(1, 0, 0, 0.5));
	tracer_cols.push_back(tVector(0, 0.5, 0, 0.5));
	tracer_cols.push_back(tVector(0.75, 0, 0.75, 0.5));
	tracer_cols.push_back(tVector(0, 0.5, 0.5, 0.5));
	tracer_cols.push_back(tVector(0, 0, 0, 0.5));
	int handle = AddCharTrace(mScene->GetCharacter(), tracer_cols);
	mTraceHandles.push_back(handle);
}

int cDrawScenarioSimChar::AddCharTrace(const std::shared_ptr<cSimCharacter>& character,
										const tVectorArr& cols)
{
	cCharTracer::tParams params;
	params.mChar = character;
	params.mColors = cols;
	params.mType = cCharTracer::eTraceCOM;

	for (int i = 0; i < character->GetNumBodyParts(); ++i)
	{
		if (character->IsValidBodyPart(i)
			&& character->IsEndEffector(i))
		{
			params.mContactList.push_back(i);
		}
	}

	int handle = mTracer.AddTrace(params);
	return handle;
}

void cDrawScenarioSimChar::ToggleTrace()
{
	mTracer.Reset();
	mEnableTrace = !mEnableTrace;
}

void cDrawScenarioSimChar::DrawScene()
{
	DrawGrid();
	DrawGroundMainScene();
	DrawCharacterMainScene();
	DrawObjsMainScene();
	DrawPerturbs();

	if (mDrawInfo)
	{
		DrawInfo();
	}

	if (mDrawPoliInfo)
	{
		DrawPoliInfo();
	}

	if (mEnableTrace)
	{
		DrawTrace();
	}

	if (mDrawFeatures)
	{
		DrawFeatures();
	}

	if (mDrawPolicyPlots)
	{
		DrawPolicyPlots();
	}
}

void cDrawScenarioSimChar::DrawGrid() const
{
	const double spacing = 0.10f;
	const double big_spacing = spacing * 5.f;
	tVector origin = mCam.GetFocus();
	origin += tVector(0, 0, -1, 0);
	tVector size = tVector(mCam.GetWidth(), mCam.GetHeight(), 0, 0);

	cDrawUtil::SetColor(tVector(188 / 255.f, 219 / 255.f, 242 / 255.f, 1.f));
	cDrawUtil::DrawGrid2D(origin, size, spacing, big_spacing);
}

void cDrawScenarioSimChar::DrawGroundMainScene()
{
	DrawGround();
}

void cDrawScenarioSimChar::DrawCharacterMainScene()
{
	DrawCharacter();
}

void cDrawScenarioSimChar::DrawObjsMainScene()
{
	DrawObjs();
}

void cDrawScenarioSimChar::DrawGround() const
{
	const auto& ground = mScene->GetGround();
	
	tVector focus = mCam.GetFocus();
	double cam_w = mCam.GetWidth();
	double cam_h = mCam.GetHeight();

	tVector ground_col = GetGroundColor();
	tVector bound_min = focus - tVector(cam_w, cam_h, 0, 0) * 0.5;
	tVector bound_max = focus + tVector(cam_w, cam_h, 0, 0) * 0.5;
	cDrawGround::Draw2D(ground.get(), ground_col, bound_min, bound_max);
}

void cDrawScenarioSimChar::DrawCharacter() const
{
	const auto& character = mScene->GetCharacter();
	bool enable_draw_shape = true;
	cDrawSimCharacter::Draw(*(character.get()), gFillTint, GetLineColor(), enable_draw_shape);
}

void cDrawScenarioSimChar::DrawTrace() const
{
	glPushMatrix();
	cDrawUtil::Translate(GetVisOffset());
	mTracer.Draw();
	glPopMatrix();
}

void cDrawScenarioSimChar::DrawObjs() const
{
	const tVector col = tVector(0.5, 0.5, 0.5, 1);

	const auto& obj_entries = mScene->GetObjs();
	for (size_t i = 0; i < obj_entries.size(); ++i)
	{
		const cScenarioSimChar::tObjEntry& entry = obj_entries[i];
		const auto& obj = entry.mObj;

		cDrawUtil::SetColor(tVector(col[0], col[1], col[2], col[3]));
		cDrawObj::Draw(obj.get(), cDrawUtil::eDrawSolid);

		tVector line_col = GetLineColor();
		if (line_col[3] > 0)
		{
			cDrawUtil::SetColor(line_col);
			cDrawObj::Draw(obj.get(), cDrawUtil::eDrawWire);
		}
	}
}

tVector cDrawScenarioSimChar::GetVisOffset() const
{
	return gVisOffset;
}

tVector cDrawScenarioSimChar::GetLineColor() const
{
	return gLineColor;
}

tVector cDrawScenarioSimChar::GetGroundColor() const
{
	return tVector(151 / 255.0, 151 / 255.0, 151 / 255.0, 1.0);
}

void cDrawScenarioSimChar::DrawInfo() const
{
	DrawCoM();
	DrawTorque();
	DrawCtrlInfo();
}

void cDrawScenarioSimChar::DrawCoM() const
{
	const tVector col = tVector(0, 1, 0, 0.5);
	const double marker_size = 0.1;
	const double vel_scale = 0.1;
	const auto& character = mScene->GetCharacter();
	cDrawSimCharacter::DrawCoM(*(character.get()), marker_size, vel_scale, col, GetVisOffset());
}

void cDrawScenarioSimChar::DrawTorque() const
{
	const auto& character = mScene->GetCharacter();
	cDrawSimCharacter::DrawTorque(*(character.get()), GetVisOffset());
}

void cDrawScenarioSimChar::DrawCtrlInfo() const
{
	const auto& character = mScene->GetCharacter();
	cDrawSimCharacter::DrawCtrlInfo(character->GetController().get(), GetVisOffset());
}

void cDrawScenarioSimChar::DrawPoliInfo() const
{
	const auto& character = mScene->GetCharacter();
	cDrawSimCharacter::DrawPoliInfo(character->GetController().get(), mCam);
}

void cDrawScenarioSimChar::DrawFeatures() const
{
	const double marker_size = 0.05;
	const double vel_scale = 0.025;
	const tVector pos_col = tVector(1, 0, 0, 0.5);
	const tVector vel_col = tVector(0, 0.75, 0, 0.5);
	const tVector terrain_col = tVector(0, 0, 1, 0.5);
	const auto& character = mScene->GetCharacter();
	const auto& ground = mScene->GetGround();

	cDrawSimCharacter::DrawCharFeatures(*(character.get()), *ground.get(), 
						marker_size, vel_scale, pos_col, vel_col, GetVisOffset());
	cDrawSimCharacter::DrawTerainFeatures(*(character.get()), marker_size, terrain_col, GetVisOffset());
}

void cDrawScenarioSimChar::DrawPolicyPlots() const
{
	const auto& character = mScene->GetCharacter();
	cDrawSimCharacter::DrawPolicyPlots(character->GetController().get(), mCam);
}

void cDrawScenarioSimChar::DrawPerturbs() const
{
	const auto& world = mScene->GetWorld();
	cDrawWorld::DrawPerturbs(*world.get());
}

std::string cDrawScenarioSimChar::BuildTextInfoStr() const
{
	const auto& character = mScene->GetCharacter();
	tVector com = character->CalcCOM();
	tVector com_vel = character->CalcCOMVel();
	char buffer[128];
#ifdef _WIN32
	sprintf_s(buffer, "Position: (%.2f, %.2f)\nVelocity: (%.2f, %.2f)\n", com[0], com[1], com_vel[0], com_vel[1]);
#else
	sprintf(buffer, "Position: (%.2f, %.2f)\nVelocity: (%.2f, %.2f)\n", com[0], com[1], com_vel[0], com_vel[1]);
#endif
	std::string str(buffer);
	return str;
}

void cDrawScenarioSimChar::Shutdown()
{
	mScene->Shutdown();
}

const std::shared_ptr<cScenarioSimChar>& cDrawScenarioSimChar::GetScene() const
{
	return mScene;
}

std::string cDrawScenarioSimChar::GetOutputCharFile() const
{
	return gOutputCharFile;
}

void cDrawScenarioSimChar::OutputCharState(const std::string& out_file) const
{
	mScene->OutputCharState(out_file);
}

void cDrawScenarioSimChar::SpawnProjectile()
{
	mScene->SpawnProjectile();
}

void cDrawScenarioSimChar::SpawnBigProjectile()
{
	mScene->SpawnBigProjectile();
}
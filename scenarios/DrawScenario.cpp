#include "DrawScenario.h"
#include "render/DrawUtil.h"

cDrawScenario::cDrawScenario(cCamera& cam)
	: mCam(cam)
{
	mCamTrackMode = eCamTrackModeX;
	mCamTrackMode0 = mCamTrackMode;
}

void cDrawScenario::ParseArgs(const cArgParser& parser)
{
	ParseCamTrackMode(parser, mCamTrackMode);
	mCamTrackMode0 = mCamTrackMode;
}

void cDrawScenario::Update(double time_elapsed)
{
	UpdateCamera();
}

void cDrawScenario::Draw()
{
	glPushMatrix();
	mCam.SetupGLView();

	DrawSetup();
	DrawScene();
	DrawCleanup();

	glPopMatrix();
}

void cDrawScenario::Reset()
{
	cScenario::Reset();
	ResetCamera();
}


void cDrawScenario::Keyboard(unsigned char key, int x, int y)
{
}

void cDrawScenario::MouseClick(int button, int state, double world_x, double world_y)
{
}

void cDrawScenario::MouseMove(double x, double y)
{
}

void cDrawScenario::Reshape(int w, int h)
{
}

cDrawScenario::eCamTrackMode cDrawScenario::GetTrackMode() const
{
	return mCamTrackMode;
}

cDrawScenario::eCamTrackMode cDrawScenario::GetTrackMode0() const
{
	return mCamTrackMode0;
}

void cDrawScenario::ParseCamTrackMode(const cArgParser& parser, eCamTrackMode& out_mode) const
{
	std::string str = "";
	parser.ParseString("cam_track_mode", str);

	if (str != "")
	{
		if (str == "x")
		{
			out_mode = eCamTrackModeX;
		}
		else if (str == "y")
		{
			out_mode = eCamTrackModeY;
		}
		else if (str == "still")
		{
			out_mode = eCamTrackModeStill;
		}
		else if (str == "fixed")
		{
			out_mode = eCamTrackModeFixed;
		}
		else
		{
			assert(false); // unsupported track mode
		}
	}
}

void cDrawScenario::UpdateCamera()
{
	if (mCamTrackMode == eCamTrackModeX
		|| mCamTrackMode == eCamTrackModeY
		|| mCamTrackMode == eCamTrackModeXYZ)
	{
		UpdateCameraTracking();
	}
	else if (mCamTrackMode == eCamTrackModeStill)
	{
		UpdateCameraStill();
	}
}

void cDrawScenario::UpdateCameraTracking()
{
	if (mCamTrackMode == eCamTrackModeXYZ)
	{
		tVector track_pos = GetCamTrackPos();
		tVector focus_pos = mCam.GetFocus();
		tVector cam_pos = mCam.GetPosition();
		mCam.TranslateFocus(track_pos);
	}
	else if (mCamTrackMode == eCamTrackModeX
		|| mCamTrackMode == eCamTrackModeY)
	{
		tVector track_pos = GetCamTrackPos();
		tVector cam_focus = mCam.GetFocus();

		double cam_w = mCam.GetWidth();
		double cam_h = mCam.GetHeight();
		const double y_pad = std::min(0.5, 0.8 * 0.5 * cam_h);
		const double x_pad = std::min(0.5, 0.8 * 0.5 * cam_w);

		if (mCamTrackMode == eCamTrackModeX)
		{
			cam_focus[0] = track_pos[0];

			if (std::abs(track_pos[1] - cam_focus[1]) > ((0.5 * cam_h) - y_pad))
			{
				const double blend = 0.5;
				double tar_y = track_pos[1] + ((0.5 * cam_h) - y_pad);
				cam_focus[1] = (1 - blend) * cam_focus[1] + blend * tar_y;
			}
		}
		else
		{
			cam_focus[1] = track_pos[1];

			const double blend = 0.5;
			double tar_delta = track_pos[0] - cam_focus[0];
			if (std::abs(tar_delta) > ((0.5 * cam_w) - x_pad))
			{
				double tar_x = track_pos[0] + cMathUtil::Sign(tar_delta) * ((0.95 * cam_w) - x_pad);
				cam_focus[0] = (1 - blend) * cam_focus[0] + blend * tar_x;
			}
		}

		mCam.TranslateFocus(cam_focus);
	}
}

void cDrawScenario::UpdateCameraStill()
{
	tVector track_pos = GetCamTrackPos();
	tVector cam_focus = mCam.GetFocus();

	double cam_w = mCam.GetWidth();
	double cam_h = mCam.GetHeight();
	double cam_still_snap_dist = GetCamStillSnapDistX();

	const double pad_x = std::min(0.5, 0.4 * cam_still_snap_dist);
	const double pad_y = std::min(0.0, 0.2 * cam_h);

	double avg_h = 0;
	bool snap_x = std::abs(track_pos[0] - cam_focus[0]) > cam_still_snap_dist - pad_x;
	bool snap_y = (track_pos[1] - cam_focus[1]) > 0.5 * cam_h - pad_y
				|| (track_pos[1] - cam_focus[1]) < -(0.5 * cam_h - pad_y);

	if (snap_x || snap_y)
	{
		tVector snap_pos = GetCamStillPos();
		cam_focus[0] = snap_pos[0];
		cam_focus[1] = snap_pos[1];

		tVector pos_delta = track_pos - snap_pos;
		if (std::abs(pos_delta[0]) > cam_still_snap_dist - pad_x)
		{
			cam_focus[0] += pos_delta[0];
		}

		if ((pos_delta[1]) > 0.5 * cam_h - pad_y
			|| (pos_delta[1]) < -(0.5 * cam_h - pad_y))
		{
			cam_focus[1] += pos_delta[1];
		}

		if (snap_x)
		{
			cam_focus[0] += cam_still_snap_dist - pad_x;
		}
	}

	mCam.TranslateFocus(cam_focus);
}

double cDrawScenario::GetCamStillSnapDistX() const
{
	const tVector axis = tVector(1, 0, 0, 0);
	double dist = 0.5 * mCam.GetWidth();

	// this is all hacks
	tVector view_delta = mCam.GetFocus() - mCam.GetPosition();
	tVector view_dir = view_delta.normalized();
	double len = view_delta.norm() - mCam.GetNearZ();
	view_delta = view_dir * len;

	view_delta[1] = 0;
	view_delta[3] = 0;
	len = view_delta.norm();

	if (len > 0)
	{
		view_delta /= len;
		double dot = view_delta.dot(axis);
		double lerp = std::abs(dot);
		lerp = std::pow(lerp, 4);
		lerp = 1 - lerp;
		dist = lerp * dist + (1 - lerp) * 0.5 * len;
	}
	return dist;
}

tVector cDrawScenario::GetCamTrackPos() const
{
	return tVector::Zero();
}

tVector cDrawScenario::GetCamStillPos() const
{
	return tVector::Zero();
}

void cDrawScenario::ToggleCamTrackMode(eCamTrackMode mode)
{
	if (mCamTrackMode == mode)
	{
		mCamTrackMode = mCamTrackMode0;
	}
	else
	{
		mCamTrackMode = mode;
	}
}

void cDrawScenario::ResetCamera()
{
	tVector target_pos = GetDefaultCamFocus();

	if (mCamTrackMode == eCamTrackModeX
		|| mCamTrackMode == eCamTrackModeY)
	{
		target_pos = GetCamTrackPos();
	}
	else if (mCamTrackMode == eCamTrackModeStill)
	{
		target_pos = GetCamStillPos();
	}

	tVector cam_pos = GetDefaultCamFocus();
	cam_pos[0] = target_pos[0];
	cam_pos[1] = target_pos[1];

	mCam.TranslateFocus(cam_pos);
}

tVector cDrawScenario::GetDefaultCamFocus() const
{
	return tVector::Zero();
}

void cDrawScenario::DrawSetup()
{
}

void cDrawScenario::DrawCleanup()
{
}

void cDrawScenario::DrawScene()
{
}

std::string cDrawScenario::BuildTextInfoStr() const
{
	return "";
}

cDrawScenario::~cDrawScenario()
{
}
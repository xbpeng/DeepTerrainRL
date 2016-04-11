#include "DrawScenarioSimInteractive.h"
#include "render/DrawUtil.h"
#include "render/DrawPerturb.h"

const double gLinkWidth = 0.05f;
const tVector gLineColor = tVector(0, 0, 0, 1);
const tVector gFillTint = tVector(1, 1, 1, 1);
const tVector gCamFocus0 = tVector(0, 0.75, 0, 0);

cDrawScenarioSimInteractive::cDrawScenarioSimInteractive(cCamera& cam)
	: cDrawScenario(cam)
{
	cam.TranslateFocus(gCamFocus0);
	ResetUI();
}

cDrawScenarioSimInteractive::~cDrawScenarioSimInteractive()
{
}

void cDrawScenarioSimInteractive::Init()
{
	cDrawScenario::Init();
	ResetUI();
}

void cDrawScenarioSimInteractive::Reset()
{
	cDrawScenario::Reset();
	ResetUI();
}

void cDrawScenarioSimInteractive::Clear()
{
	cDrawScenario::Clear();
	ResetUI();
}

void cDrawScenarioSimInteractive::Update(double time_elapsed)
{
	cDrawScenario::Update(time_elapsed);
	ApplyUIForce(time_elapsed);
}

void cDrawScenarioSimInteractive::MouseClick(int button, int state, double x, double y)
{
	const double ray_max_dist = 1000;
 	cDrawScenario::MouseClick(button, state, x, y);
	
	if (button == GLUT_LEFT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			mClickScreenPos = tVector(x, y, 0, 0);
			mDragScreenPos = mClickScreenPos;
			tVector start = mCam.ScreenToWorldPos(mClickScreenPos);
			tVector dir = mCam.GetRayCastDir(start);
			tVector end = start + dir * ray_max_dist;

			tVector intersection;
			mSelectedObj = RayTest(start, end, intersection);

			if (ObjectSelected())
			{
				mSelectObjLocalPos = mSelectedObj->WorldToLocalPos(intersection);
			}
		}
		else if (state == GLUT_UP)
		{
			ResetUI();
		}
	}
}

void cDrawScenarioSimInteractive::MouseMove(double x, double y)
{
	cDrawScenario::MouseMove(x, y);

	if (ObjectSelected())
	{
		mDragScreenPos = tVector(x, y, 0, 0);
	}
}

void cDrawScenarioSimInteractive::ResetUI()
{
	mClickScreenPos.setZero();
	mDragScreenPos.setZero();
	mSelectObjLocalPos.setZero();
	mSelectedObj = nullptr;
}


tVector cDrawScenarioSimInteractive::GetDefaultCamFocus() const
{
	return gCamFocus0;
}

cSimObj* cDrawScenarioSimInteractive::RayTest(const tVector& start, const tVector& end, tVector& out_intersection)
{
	return GetScene()->RayTest(start, end, out_intersection);
}

bool cDrawScenarioSimInteractive::ObjectSelected() const
{
	return mSelectedObj != nullptr;
}

void cDrawScenarioSimInteractive::ApplyUIForce(double time_step)
{
	if (ObjectSelected())
	{
		const double drag_threshold = 0.01;
		const double force_scale = 1 / cDrawPerturb::gForceScale;
		if ((mClickScreenPos - mDragScreenPos).squaredNorm() > drag_threshold)
		{
			tVector start = mCam.ScreenToWorldPos(mClickScreenPos);
			tVector end = mCam.ScreenToWorldPos(mDragScreenPos);
			start = mCam.ProjectToFocalPlane(start);
			end = mCam.ProjectToFocalPlane(end);

			tVector force = end - start;
			force *= force_scale;
			
			tPerturb perturb = tPerturb(tPerturb::ePerturbForce, mSelectedObj, mSelectObjLocalPos,
								force, time_step);
			GetScene()->AddPerturb(perturb);
		}
	}
}
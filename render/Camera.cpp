#include "Camera.h"
#include <iostream>
// TODO: pull out into some util/platform header file.
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

cCamera::cCamera()
{
	mPosition = tVector(0, 0, 1, 0);
	mFocus = tVector(0, 0, 0, 0);
	mUp = tVector(0, 1, 0, 0);
	mNearZ = 0;
	mFarZ = 1;

	mProj = eProjPerspecctive;

	mMouseDown = 0;
	mMousePos = tVector::Zero();

	Resize(1, 1);
	SetProjFocalLen((mFocus - mPosition).norm());
}

cCamera::~cCamera()
{
}

cCamera::cCamera(const tVector& pos, const tVector& focus, const tVector& up,
				double w, double h, double near_z, double far_z)
{
	mPosition = pos;
	mFocus = focus;
	mUp = up;
	mNearZ = near_z;
	mFarZ = far_z;
	
	Resize(w, h);

	mMouseDown = 0;
	mMousePos = tVector::Zero();
	SetProjFocalLen((mFocus - mPosition).norm());
}

const tVector& cCamera::GetPosition() const
{
	return mPosition;
}

const tVector& cCamera::GetFocus() const
{
	return mFocus;
}

const tVector& cCamera::GetUp() const
{
	return mUp;
}

tVector cCamera::GetViewDir() const
{
	tVector dir = mFocus - mPosition;
	dir[3] = 0;
	dir = dir.normalized();
	return dir;
}


double cCamera::GetWidth() const
{
	return mWidth;
}
double cCamera::GetHeight() const
{
	return mWidth / mAspectRatio;
}

double cCamera::GetAspectRatio() const
{
	return mAspectRatio;
}


double cCamera::GetNearZ() const
{
	return mNearZ;
}

double cCamera::GetFarZ() const
{
	return mFarZ;
}

double cCamera::CalcFOV() const
{
	double focal_len = mProjFocalLen;
	double h = GetHeight();
	double fov = std::atan(h * 0.5 / focal_len) * 2;
	return fov;
}


void cCamera::SetPosition(const tVector& pos)
{
	mPosition = pos;
}

void cCamera::SetFocus(const tVector& focus)
{
	mFocus = focus;
}

void cCamera::SetUp(const tVector& up)
{
	mUp = up;
}

void cCamera::Resize(double w, double h)
{
	mWidth = w;
	mAspectRatio = w / h;
}

void cCamera::SetProjFocalLen(double len)
{
	mProjFocalLen = len;
}

void cCamera::TranslatePos(const tVector& pos)
{
	tVector delta = pos - mPosition;
	mFocus += delta;
	mPosition = pos;
}

void cCamera::TranslateFocus(const tVector& focus)
{
	tVector delta = focus - mFocus;
	mPosition += delta;
	mFocus = focus;
}

// screent pos normalized between [-1, 1] with positive y pointing up
tVector cCamera::ScreenToWorldPos(const tVector& screen_pos) const
{
	tMatrix T = BuildViewWorldMatrix();
	double w = GetWidth();
	double h = GetHeight();

	double focal_len = CalcFocalLen();
	tVector view_pos = tVector(0.5 * screen_pos[0] * w,
								0.5 * screen_pos[1] * h,
								-focal_len, 1);
	tVector world_pos = T * view_pos;

	tVector dir = GetRayCastDir(world_pos);
	world_pos += -dir * (focal_len - mNearZ);
	world_pos[3] = 0;
	return world_pos;
}

tVector cCamera::ProjectToFocalPlane(const tVector& world_pos) const
{
	tVector dir = GetRayCastDir(world_pos);
	tVector origin = mFocus;
	tVector norm = GetViewDir();
	tVector delta = world_pos - origin;
	double t = (origin - world_pos).dot(norm) / dir.dot(norm);
	tVector proj_pos = t * dir + world_pos;
	return proj_pos;
}

tMatrix cCamera::BuildViewWorldMatrix() const
{
	tVector up = GetUp();
	const tVector& forward = GetViewDir();
	tVector left = up.cross3(forward).normalized();
	up = -left.cross3(forward).normalized();
	const tVector& pos = GetPosition();

	tMatrix T;
	T.col(0) = -left;
	T.col(1) = up;
	T.col(2) = -forward;
	T.col(3) = pos;
	T(3, 3) = 1;

	return T;
}

tMatrix cCamera::BuildWorldViewMatrix() const
{
	tMatrix view_world = BuildViewWorldMatrix();
	tMatrix world_view = cMathUtil::InvRigidMat(view_world);
	return world_view;
}

tMatrix cCamera::BuildProjMatrix() const
{
	tVector cam_focus = GetFocus();
	tVector cam_pos = GetPosition();
	double w = GetWidth();
	double h = GetHeight();
	double near_z = GetNearZ();
	double far_z = GetFarZ();
	double aspect = GetAspectRatio();

	tMatrix proj_mat;
	switch (mProj)
	{
	case eProjPerspecctive:
		proj_mat = BuildProjMatrixProj();
		break;
	case eProjOrtho:
		proj_mat = BuildProjMatrixOrtho();
		break;
	default:
		assert(false); // unsupported projection
		break;
	}
	return proj_mat;
}

void cCamera::SetProj(eProj proj)
{
	mProj = proj;
}

cCamera::eProj cCamera::GetProj() const
{
	return mProj;
}

void cCamera::SetupGLView() const
{
	GLint prev_mode;
	glGetIntegerv(GL_MATRIX_MODE, &prev_mode);
	glMatrixMode(GL_MODELVIEW);

	tMatrix world_view = BuildWorldViewMatrix();
	glLoadMatrixd(world_view.data());

	glMatrixMode(prev_mode);
}

void cCamera::SetupGLProj() const
{
	GLint prev_mode;
	glGetIntegerv(GL_MATRIX_MODE, &prev_mode);
	glMatrixMode(GL_PROJECTION);

	tMatrix proj_mat = BuildProjMatrix();
	glLoadMatrixd(proj_mat.data());

	glMatrixMode(prev_mode);
}

tVector cCamera::GetRayCastDir(const tVector& pos) const
{
	tVector dir = tVector::Zero();
	switch (mProj)
	{
	case eProjPerspecctive:
		dir = pos - mPosition;
		break;
	case eProjOrtho:
		dir = mFocus - mPosition;
		break;
	default:
		assert(false); // unsupported projection
		break;
	}
	dir[3] = 0;
	dir = dir.normalized();
	return dir;
}

void cCamera::MouseClick(int button, int state, double x, double y)
{
	mMouseDown = (button == GLUT_RIGHT_BUTTON) && (state == GLUT_DOWN);
	mMousePos[0] = x;
	mMousePos[1] = y;
}

void cCamera::MouseMove(double x, double y)
{
	if (mMouseDown)
	{
		int mouse_mod = glutGetModifiers();
		double w = GetWidth();
		double h = GetHeight();

		double dx = x - mMousePos[0];
		double dy = y - mMousePos[1];

		tVector cam_offset = mPosition - mFocus;
		tVector cam_dir = -cam_offset.normalized();
		tVector right = -tVector(0, 1, 0, 0).cross3(cam_dir).normalized();
		tVector up = right.cross3(cam_dir).normalized();

		if (mouse_mod & GLUT_ACTIVE_ALT) {
			tVector delta = cam_offset * (1 - dx);
			mPosition = mFocus + delta;
		}
		else if (mouse_mod & GLUT_ACTIVE_SHIFT) {
			tVector delta = 0.5 * (-w * right * dx - h * up * dy);
			mPosition += delta;
			mFocus += delta;
		}
		else
		{
			cam_offset = cMathUtil::RotateMat(tVector(0, 1, 0, 0), -M_PI * dx) * cam_offset;
			cam_offset = cMathUtil::RotateMat(right, M_PI * dy) * cam_offset;
			mPosition = mFocus + cam_offset;
		}

		// Remember mouse coords for next time.
		mMousePos[0] = x;
		mMousePos[1] = y;
	}
}

double cCamera::CalcFocalLen() const
{
	return (mFocus - mPosition).norm();
}

tMatrix cCamera::BuildProjMatrixOrtho() const
{
	tMatrix mat = tMatrix::Identity();
	double w = GetWidth();
	double h = GetHeight();

	mat(0, 0) = 2 / w;
	mat(1, 1) = 2 / h;
	mat(2, 2) = -2 / (mFarZ - mNearZ);
	mat(2, 3) = -(mFarZ + mNearZ) / (mFarZ - mNearZ);

	return mat;
}

tMatrix cCamera::BuildProjMatrixProj() const
{
	tMatrix mat = tMatrix::Zero();
	double focal_len = mProjFocalLen;
	double w = GetWidth();
	double h = GetHeight();

	mat(0, 0) = 2 * focal_len / w;
	mat(1, 1) = 2 * focal_len / h;
	mat(2, 2) = -(mFarZ + mNearZ) / (mFarZ - mNearZ);
	mat(2, 3) = -2 * mFarZ * mNearZ / (mFarZ - mNearZ);
	mat(3, 2) = -1;

	return mat;
}
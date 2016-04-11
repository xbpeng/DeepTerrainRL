#pragma once

#include "util/MathUtil.h"

class cCamera
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eProj
	{
		eProjPerspecctive,
		eProjOrtho,
		eProjMax
	};

	cCamera();
	cCamera(const tVector& pos, const tVector& focus, const tVector& up,
		double w, double h, double near_z, double far_z);
	virtual ~cCamera();

	virtual const tVector& GetPosition() const;
	virtual const tVector& GetFocus() const;
	virtual const tVector& GetUp() const;
	virtual tVector GetViewDir() const;
	virtual double GetWidth() const;
	virtual double GetHeight() const;
	virtual double GetAspectRatio() const;
	virtual double GetNearZ() const;
	virtual double GetFarZ() const;
	virtual double CalcFOV() const;

	virtual void SetPosition(const tVector& pos);
	virtual void SetFocus(const tVector& focus);
	virtual void SetUp(const tVector& up);
	virtual void Resize(double w, double h);
	virtual void SetProjFocalLen(double len);

	virtual void TranslatePos(const tVector& pos);
	virtual void TranslateFocus(const tVector& focus);

	virtual tVector ScreenToWorldPos(const tVector& screen_pos) const;
	virtual tVector ProjectToFocalPlane(const tVector& world_pos) const;
	virtual tMatrix BuildViewWorldMatrix() const;
	virtual tMatrix BuildWorldViewMatrix() const;
	virtual tMatrix BuildProjMatrix() const;

	virtual void SetProj(eProj proj);
	virtual eProj GetProj() const;

	virtual void SetupGLView() const;
	virtual void SetupGLProj() const;

	virtual tVector GetRayCastDir(const tVector& pos) const;

	virtual void MouseClick(int button, int state, double x, double y);
	virtual void MouseMove(double x, double y);

protected:
	eProj mProj;

	tVector mPosition;
	tVector mFocus;
	tVector mUp;

	double mWidth;
	double mAspectRatio;
	double mNearZ;
	double mFarZ;

	bool mMouseDown;
	tVector mMousePos;

	double mProjFocalLen;

	virtual double CalcFocalLen() const;
	virtual tMatrix BuildProjMatrixOrtho() const;
	virtual tMatrix BuildProjMatrixProj() const;
};
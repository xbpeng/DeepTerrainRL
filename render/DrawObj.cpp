#include "DrawObj.h"

void cDrawObj::Draw(const cSimObj* obj, cDrawUtil::eDrawMode draw_mode)
{
	cSimObj::eShape shape = obj->GetShape();
	switch (shape)
	{
	case cSimObj::eShapeBox:
		DrawBox(reinterpret_cast<const cSimBox*>(obj), draw_mode);
		break;
	case cSimObj::eShapePlane:
		DrawPlane(reinterpret_cast<const cSimPlane*>(obj), draw_mode);
		break;
	case cSimObj::eShapeCapsule:
		DrawCapsule(reinterpret_cast<const cSimCapsule*>(obj), draw_mode);
		break;
	default:
		assert(false); // unsupported shape
		break;
	}
}

void cDrawObj::DrawBox(const cSimBox* box, cDrawUtil::eDrawMode draw_mode)
{
	tVector pos = box->GetPos();
	tVector size = box->GetSize();
	tVector axis;
	double theta;
	box->GetRotation(axis, theta);

	glPushMatrix();

	cDrawUtil::Translate(pos);
	cDrawUtil::Rotate(theta, axis);
	cDrawUtil::DrawBox(tVector::Zero(), size, draw_mode);

	glPopMatrix();
}

void cDrawObj::DrawPlane(const cSimPlane* plane, double size, cDrawUtil::eDrawMode draw_mode)
{
	tVector coeffs = plane->GetCoeffs();
	cDrawUtil::DrawPlane(coeffs, size, draw_mode);
}

void cDrawObj::DrawCapsule(const cSimCapsule* cap, cDrawUtil::eDrawMode draw_mode)
{
	tVector pos = cap->GetPos();
	double h = cap->GetHeight();
	double r = cap->GetRadius();
	tVector axis;
	double theta;
	cap->GetRotation(axis, theta);

	glPushMatrix();

	cDrawUtil::Translate(pos);
	cDrawUtil::Rotate(theta, axis);
	cDrawUtil::DrawCapsule(h, r, 8, 8, draw_mode);

	glPopMatrix();
}
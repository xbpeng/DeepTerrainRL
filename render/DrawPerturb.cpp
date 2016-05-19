#include "DrawPerturb.h"
#include "render/DrawUtil.h"
#include "sim/SimObj.h"

const double cDrawPerturb::gForceScale = 0.01;
const double cDrawPerturb::gTorqueScale = 0.00075;// * 0.25;

void cDrawPerturb::DrawForce2D(const tVector& pos, const tVector& force)
{
	const double len_scale = gForceScale;
	const double arrow_size = 0.075;
	tVector pos1 = pos + force * len_scale;

	cDrawUtil::SetColor(tVector(1, 0, 0, 0.5));
	cDrawUtil::DrawArrow2D(pos, pos1, arrow_size);
}

void cDrawPerturb::DrawTorque2D(const tVector& pos, const tVector& torque)
{
	const double torque_scale = gTorqueScale;
	const int slices = 16;
	const tVector color0 = tVector(1, 0, 0, 0.25);
	const tVector color1 = tVector(0, 1, 1, 0.25);

	tVector col = (torque[2] < 0) ? color0 : color1;

	double mag = torque.norm();
	double r = mag * torque_scale;

	cDrawUtil::SetColor(tVector(col[0], col[1], col[2], col[3]));

	glPushMatrix();
	cDrawUtil::Translate(pos);
	cDrawUtil::DrawDisk(r, slices);
	glPopMatrix();
}

void cDrawPerturb::Draw(const tPerturb& perturb)
{
	tPerturb::ePerturb type = perturb.mType;
	switch (type)
	{
	case tPerturb::ePerturbForce:
		DrawForce(perturb);
		break;
	case tPerturb::ePerturbTorque:
		DrawTorque(perturb);
		break;
	default:
		break;
	}
}

void cDrawPerturb::DrawForce(const tPerturb& perturb)
{
	tVector pos = perturb.mObj->LocalToWorldPos(perturb.mLocalPos);
	const tVector& force = perturb.mPerturb;
	DrawForce2D(pos, force);
}

void cDrawPerturb::DrawTorque(const tPerturb& perturb)
{
	tVector pos = perturb.mObj->LocalToWorldPos(perturb.mLocalPos);
	const tVector& torque = perturb.mPerturb;
	DrawTorque2D(pos, torque);
}
#pragma once

#include "DrawUtil.h"
#include "util/MathUtil.h"
#include "sim/SimBox.h"
#include "sim/SimPlane.h"
#include "sim/SimCapsule.h"

class cDrawObj
{
public:
	static void Draw(const cSimObj* obj, cDrawUtil::eDrawMode draw_mode = cDrawUtil::eDrawSolid);
	static void DrawBox(const cSimBox* box, cDrawUtil::eDrawMode draw_mode = cDrawUtil::eDrawSolid);
	static void DrawPlane(const cSimPlane* plane, double size, cDrawUtil::eDrawMode draw_mode = cDrawUtil::eDrawSolid);
	static void DrawCapsule(const cSimCapsule* cap, cDrawUtil::eDrawMode draw_mode = cDrawUtil::eDrawSolid);
};
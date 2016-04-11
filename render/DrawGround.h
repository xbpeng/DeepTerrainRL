#pragma once

#include "sim/Ground.h"

class cDrawGround
{
public:
	static void Draw2D(const cGround* ground, const tVector& col, const tVector& bound_min, const tVector& bound_max);
	static void Draw3D(const cGround* ground, const tVector& col, const tVector& bound_min, const tVector& bound_max);

protected:
	static void DrawFlat2D(const cGround* ground, const tVector& col, const tVector& bound_min, const tVector& bound_max);
	static void DrawFlat3D(const cGround* ground, const tVector& col, const tVector& bound_min, const tVector& bound_max);
	static void DrawVar2D(const cGround* ground, const tVector& col, const tVector& bound_min, const tVector& bound_max);
	static void DrawVar3D(const cGround* ground, const tVector& col, const tVector& bound_min, const tVector& bound_max);
};
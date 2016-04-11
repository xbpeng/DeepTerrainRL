#include "DrawGround.h"
#include "DrawUtil.h"
#include "sim/GroundFlat.h"
#include "sim/GroundVar2D.h"

const double gMarkerSpacing = 0.20;
const double gBigMarkerSpacing = gMarkerSpacing * 5;
const double gMarkerH = 0.04;
const double gBigMarkerH = 0.075;

void cDrawGround::Draw2D(const cGround* ground, const tVector& col, const tVector& bound_min, const tVector& bound_max)
{
	cGround::eGroundType type = ground->GetGroundType();
	switch (type)
	{
	case cGround::eGroundTypeFlat:
		DrawFlat2D(ground, col, bound_min, bound_max);
		break;
	case cGround::eGroundTypeVar2D:
		DrawVar2D(ground, col, bound_min, bound_max);
		break;
	default:
		assert(false); // unsupported ground type
		break;
	}
}

void cDrawGround::Draw3D(const cGround* ground, const tVector& col, const tVector& bound_min, const tVector& bound_max)
{
	cGround::eGroundType type = ground->GetGroundType();
	switch (type)
	{
	case cGround::eGroundTypeFlat:
		DrawFlat3D(ground, col, bound_min, bound_max);
		break;
	case cGround::eGroundTypeVar2D:
		DrawVar3D(ground, col, bound_min, bound_max);
		break;
	default:
		assert(false); // unsupported ground type
		break;
	}
}

void cDrawGround::DrawFlat2D(const cGround* ground, const tVector& col, const tVector& bound_min, const tVector& bound_max)
{
	assert(ground->GetGroundType() == cGround::eGroundTypeFlat);
	
	const cGroundFlat* ground_flat = reinterpret_cast<const cGroundFlat*>(ground);
	tVector ground_origin = ground_flat->GetPos();

	const double ground_h = ground_flat->SampleHeight(tVector::Zero());

	tVector origin = (bound_min + bound_max) * 0.5;
	double w = bound_max[0] - bound_min[0];
	double h = bound_max[1] - bound_min[1];

	double max_x = origin(0) + w;
	double max_y = ground_h;
	double min_x = origin(0) - w;
	double min_y = std::min(origin(1) - h * 0.5f, max_y - 0.05f);

	tVector pos = tVector(origin(0), (min_y + max_y) * 0.5, 0, 0);
	tVector size = tVector(w, (max_y - min_y), 0, 0);

	cDrawUtil::DrawRuler2D(pos, size, col, gMarkerSpacing, gBigMarkerSpacing, gMarkerH, gBigMarkerH);
}

void cDrawGround::DrawFlat3D(const cGround* ground, const tVector& col, const tVector& bound_min, const tVector& bound_max)
{
	assert(ground->GetGroundType() == cGround::eGroundTypeFlat);

	const cGroundFlat* ground_flat = reinterpret_cast<const cGroundFlat*>(ground);
	tVector pos = ground_flat->GetPos();
	tVector size = bound_max - bound_min;

	cDrawUtil::SetColor(col);
	glPushMatrix();
	cDrawUtil::Rotate(-0.5 * M_PI, tVector(1, 0, 0, 0));
	cDrawUtil::DrawRect(pos, size, cDrawUtil::eDrawSolid);
	glPopMatrix();
}

void cDrawGround::DrawVar2D(const cGround* ground, const tVector& col, const tVector& bound_min, const tVector& bound_max)
{
	assert(ground->GetGroundType() == cGround::eGroundTypeVar2D);

	const cGroundVar2D* ground_var = reinterpret_cast<const cGroundVar2D*>(ground);
	tVector ground_origin = ground_var->GetPos();

	tVector origin = (bound_min + bound_max) * 0.5;
	double w = bound_max[0] - bound_min[0];
	double h = bound_max[1] - bound_min[1];

	double max_x = origin(0) + w * 0.5;
	double min_x = origin(0) - w * 0.5;
	double min_y = origin(1) - h * 0.5;

	int grid_w = ground_var->GetGridWidth();

	tVector min_coord = ground_var->CalcGridCoord(tVector(min_x, 0, ground_origin[2], 0));
	tVector max_coord = ground_var->CalcGridCoord(tVector(max_x, 0, ground_origin[2], 0));
	int min_i = static_cast<int>(std::floor(min_coord[0]));
	int max_i = static_cast<int>(std::ceil(max_coord[0])) + 1; // + 1 padding

	min_i = cMathUtil::Clamp(min_i, 0, grid_w - 1);
	max_i = cMathUtil::Clamp(max_i, 0, grid_w - 1);

	for (int i = min_i; i < max_i; ++i)
	{
		tVector a = ground_var->GetVertex(i, 0);
		tVector b = ground_var->GetVertex(i + 1, 0);
		a[2] = 0;
		b[2] = 0;

		double curr_min_y = std::min(a[1], b[1]);
		curr_min_y = std::min(curr_min_y, min_y);

		tVector c = b;
		tVector d = a;
		c[1] = curr_min_y;
		d[1] = curr_min_y;
		
		cDrawUtil::SetLineWidth(1);
		cDrawUtil::SetColor(col);
		cDrawUtil::DrawQuad(a, d, c, b);
		cDrawUtil::SetColor(tVector(0, 0, 0, 1));
		cDrawUtil::DrawLine(a, b);
	}

	// draw markers
	cDrawUtil::SetColor(tVector(0.f, 0.f, 0.f, 1.f));
	for (int i = 0; i < 2; ++i)
	{
		bool big = (i == 1);
		cDrawUtil::SetLineWidth((big) ? 3.f : 2.f);
		double curr_spacing = (big) ? gBigMarkerSpacing : gMarkerSpacing;
		double curr_h = (big) ? gBigMarkerH : gMarkerH;

		for (double x = min_x - std::fmod(min_x, curr_spacing); x < max_x; x += curr_spacing)
		{
			bool valid_sample = true;
			double ground_h = ground->SampleHeight(tVector(x, 0, ground_origin[2], 0), valid_sample);
			
			if (valid_sample)
			{
				tVector a = tVector(x, ground_h + curr_h * 0.5f, 0, 0);
				tVector b = tVector(x, ground_h - curr_h * 0.5f, 0, 0);
				cDrawUtil::DrawLine(a, b);
			}
		}
	}
}

void cDrawGround::DrawVar3D(const cGround* ground, const tVector& col, const tVector& bound_min, const tVector& bound_max)
{
	assert(ground->GetGroundType() == cGround::eGroundTypeVar2D);
	const tVector tex_size = tVector(0.5, 0.5, 0, 0);

	const cGroundVar2D* ground_var = reinterpret_cast<const cGroundVar2D*>(ground);
	double ground_w = ground_var->GetWidth();
	tVector ground_origin = ground_var->GetPos();

	tVector origin = (bound_min + bound_max) * 0.5;
	double w = bound_max[0] - bound_min[0];
	double h = bound_max[1] - bound_min[1];

	double max_x = origin(0) + w * 0.5;
	double min_x = origin(0) - w * 0.5;
	double min_y = origin(1) - h * 0.5;

	int grid_w = ground_var->GetGridWidth();

	tVector min_coord = ground_var->CalcGridCoord(tVector(min_x, 0, ground_origin[2], 0));
	tVector max_coord = ground_var->CalcGridCoord(tVector(max_x, 0, ground_origin[2], 0));
	int min_i = static_cast<int>(std::floor(min_coord[0]));
	int max_i = static_cast<int>(std::ceil(max_coord[0])) + 1; // + 1 padding

	min_i = cMathUtil::Clamp(min_i, 0, grid_w - 1);
	max_i = cMathUtil::Clamp(max_i, 0, grid_w - 1);

	cDrawUtil::SetColor(col);
	for (int i = min_i; i < max_i; ++i)
	{
		tVector a = ground_var->GetVertex(i, 0);
		tVector b = ground_var->GetVertex(i + 1, 0);

		double curr_min_y = std::min(a[1], b[1]);
		curr_min_y = std::min(curr_min_y, min_y);

		tVector c = b;
		tVector d = a;
		c[1] = curr_min_y;
		d[1] = curr_min_y;

		double z0 = ground_origin[3] - 0.5 * ground_w;
		double z1 = ground_origin[3] + 0.5 * ground_w;
		double a_coord = a[0] / tex_size[0];
		double b_coord = b[0] / tex_size[0];
		b_coord -= a_coord;
		a_coord = a_coord - static_cast<int>(a_coord);
		b_coord += a_coord;

		double slope = std::abs((b[1] - a[1]) / (b[0] - a[0]));
		if (slope > 1)
		{
			a_coord = 0;
			b_coord = 0;
		}

		tVector coord_a0 = tVector(a_coord, z0 / tex_size[1], 0, 0);
		tVector coord_b0 = tVector(b_coord, z0 / tex_size[1], 0, 0);
		tVector coord_a1 = tVector(a_coord, z1 / tex_size[1], 0, 0);
		tVector coord_b1 = tVector(b_coord, z1 / tex_size[1], 0, 0);

		//tVector coord_c = tVector(b_coord, 0.2 / tex_size[1], 0, 0);
		//tVector coord_d = tVector(a_coord, 0.2 / tex_size[1], 0, 0);
		tVector coord_c = tVector(0, 0, 0, 0);
		tVector coord_d = tVector(0, 0, 0, 0);

		a[2] = z0;
		b[2] = z0;
		c[2] = z0;
		d[2] = z0;
		cDrawUtil::DrawQuad(a, b, c, d, coord_d, coord_c, coord_c, coord_d);

		a[2] = z1;
		b[2] = z1;
		c[2] = z1;
		d[2] = z1;
		cDrawUtil::DrawQuad(a, d, c, b, coord_d, coord_d, coord_c, coord_c);

		cDrawUtil::DrawQuad(tVector(a[0], a[1], z0, 0),
							tVector(a[0], a[1], z1, 0),
							tVector(b[0], b[1], z1, 0),
							tVector(b[0], b[1], z0, 0),
							coord_a0, coord_a1, coord_b1, coord_b0);
	}
}
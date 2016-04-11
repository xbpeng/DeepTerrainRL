#pragma once

#include "util/MathUtil.h"
#include "TextureDesc.h"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include <GL/glut.h>
#endif

class cDrawUtil
{
public:
	enum eDrawMode
	{
		eDrawSolid,
		eDrawWire,
		eDrawMax
	};

	static void InitDrawUtil();
	static void DrawRect(const tVector& pos, const tVector& size, eDrawMode draw_mode = eDrawSolid);
	static void DrawBox(const tVector& pos, const tVector& size, eDrawMode draw_mode = eDrawSolid);
	static void DrawTriangle(const tVector& pos, double side_len, eDrawMode draw_mode = eDrawSolid);
	static void DrawQuad(const tVector& a, const tVector& b, const tVector& c, const tVector& d, eDrawMode draw_mode = eDrawSolid);
	static void DrawQuad(const tVector& a, const tVector& b, const tVector& c, const tVector& d, 
						const tVector& coord_a, const tVector& coord_b, const tVector& coord_c, const tVector& coord_d,
						eDrawMode draw_mode = eDrawSolid);
	static void DrawDisk(const tVector& pos, double r, int slices, eDrawMode draw_mode = eDrawSolid);
	static void DrawDisk(double r, int slices, eDrawMode draw_mode = eDrawSolid);
	static void DrawPoint(const tVector& pt);
	static void DrawLine(const tVector& a, const tVector& b);
	static void DrawLineStrip(const tVectorArr& pts);
	static void DrawStrip(const tVector& a, const tVector& b, double width, eDrawMode draw_mode = eDrawSolid);
	static void DrawCross(const tVector& pos, double size);
	static void DrawPlane(const tVector& coeffs, double size, eDrawMode draw_mode = eDrawSolid);
	static void DrawSphere(double r, int slices, int stacks, eDrawMode draw_mode = eDrawSolid);
	static void DrawCylinder(double h, double r, int slices, eDrawMode draw_mode = eDrawSolid);
	static void DrawCapsule(double h, double r, int slices, int stacks, eDrawMode draw_mode = eDrawSolid);

	static void DrawArrow2D(const tVector& start, const tVector& end, double head_size);
	static void DrawGrid2D(const tVector& origin, const tVector& size, double spacing, double big_spacing);
	static void DrawRuler2D(const tVector& origin, const tVector& size,
							const tVector& col, double marker_spacing, double big_marker_spacing,
							double marker_h, double big_marker_h);

	static void DrawSemiCircle(const tVector& pos, double r, int slices, double min_theta, double max_theta, 
								eDrawMode draw_mode = eDrawSolid);
	static void DrawCalibMarker(const tVector& pos, double r, int slices, 
								const tVector& col0, const tVector& col1, eDrawMode draw_mode = eDrawSolid);

	static void DrawString(const std::string& str, const tVector& scale);
	static void DrawTexQuad(const cTextureDesc& tex, const tVector& pos, const tVector& size);
	static void CopyTexture(const cTextureDesc& src_tex, const cTextureDesc& dst_tex);

	static void ClearColor(const tVector& col);
	static void ClearDepth(double depth);

	static void Translate(const tVector& trans);
	static void Scale(const tVector& scale);
	static void Rotate(double theta, const tVector& axis);
	static void SetColor(const tVector& col);
	static void SetLineWidth(double w);
	static void SetPointSize(double pt_size);
	static void GLMultMatrix(const tMatrix& mat);

	static void Finish();
};
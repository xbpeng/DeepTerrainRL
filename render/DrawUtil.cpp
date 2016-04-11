#include "DrawUtil.h"
#include <GL/glew.h>

GLUquadricObj* gQuadObj;

void cDrawUtil::InitDrawUtil()
{
	if (gQuadObj == NULL)
	{
		gQuadObj = gluNewQuadric();
	}

	glEnable(GL_TEXTURE_2D);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glFrontFace(GL_CCW);
}

void cDrawUtil::DrawRect(const tVector& pos, const tVector& size, eDrawMode draw_mode)
{
	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_QUADS : GL_LINE_LOOP;
	tVector a = tVector(pos[0] - 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2], 0);
	tVector b = tVector(pos[0] + 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2], 0);
	tVector c = tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2], 0);
	tVector d = tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2], 0);
	DrawQuad(a, b, c, d, draw_mode);
}

void cDrawUtil::DrawBox(const tVector& pos, const tVector& size, eDrawMode draw_mode)
{
	tVector sw0 = tVector(pos[0] - 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] - 0.5 * size[2], pos[2]);
	tVector se0 = tVector(pos[0] + 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] - 0.5 * size[2], pos[2]);
	tVector ne0 = tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] - 0.5 * size[2], pos[2]);
	tVector nw0 = tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] - 0.5 * size[2], pos[2]);

	tVector sw1 = tVector(pos[0] - 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] + 0.5 * size[2], pos[2]);
	tVector se1 = tVector(pos[0] + 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2] + 0.5 * size[2], pos[2]);
	tVector ne1 = tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] + 0.5 * size[2], pos[2]);
	tVector nw1 = tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2] + 0.5 * size[2], pos[2]);

	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_QUADS : GL_LINE_LOOP;
	glTexCoord2d(0, 0);
	glBegin(gl_mode);
		// top
		glNormal3d(0, 1, 0);
		glTexCoord2d(0, 0);
		glVertex3d(nw1[0], nw1[1], nw1[2]);
		glTexCoord2d(1, 0);
		glVertex3d(ne1[0], ne1[1], ne1[2]);
		glTexCoord2d(1, 1);
		glVertex3d(ne0[0], ne0[1], ne0[2]);
		glTexCoord2d(0, 1);
		glVertex3d(nw0[0], nw0[1], nw0[2]);
	glEnd();
	glBegin(gl_mode);
		// bottom
		glNormal3d(0, -1, 0);
		glTexCoord2d(0, 0);
		glVertex3d(sw0[0], sw0[1], sw0[2]);
		glTexCoord2d(1, 0);
		glVertex3d(se0[0], se0[1], se0[2]);
		glTexCoord2d(1, 1);
		glVertex3d(se1[0], se1[1], se1[2]);
		glTexCoord2d(0, 1);
		glVertex3d(sw1[0], sw1[1], sw1[2]);
	glEnd();
	glBegin(gl_mode);
		// front
		glNormal3d(0, 0, 1);
		glTexCoord2d(0, 0);
		glVertex3d(sw1[0], sw1[1], sw1[2]);
		glTexCoord2d(1, 0);
		glVertex3d(se1[0], se1[1], se1[2]);
		glTexCoord2d(1, 1);
		glVertex3d(ne1[0], ne1[1], ne1[2]);
		glTexCoord2d(0, 1);
		glVertex3d(nw1[0], nw1[1], nw1[2]);
	glEnd();
	glBegin(gl_mode);
		// back
		glNormal3d(0, 0, -1);
		glTexCoord2d(0, 0);
		glVertex3d(se0[0], se0[1], se0[2]);
		glTexCoord2d(1, 0);
		glVertex3d(sw0[0], sw0[1], sw0[2]);
		glTexCoord2d(1, 1);
		glVertex3d(nw0[0], nw0[1], nw0[2]);
		glTexCoord2d(0, 1);
		glVertex3d(ne0[0], ne0[1], ne0[2]);
	glEnd();
	glBegin(gl_mode);
		// left
		glNormal3d(-1, 0, 0);
		glTexCoord2d(0, 0);
		glVertex3d(sw0[0], sw0[1], sw0[2]);
		glTexCoord2d(1, 0);
		glVertex3d(sw1[0], sw1[1], sw1[2]);
		glTexCoord2d(1, 1);
		glVertex3d(nw1[0], nw1[1], nw1[2]);
		glTexCoord2d(0, 1);
		glVertex3d(nw0[0], nw0[1], nw0[2]);
	glEnd();
	glBegin(gl_mode);
		// right
		glNormal3d(1, 0, 0);
		glTexCoord2d(0, 0);
		glVertex3d(se1[0], se1[1], se1[2]);
		glTexCoord2d(1, 0);
		glVertex3d(se0[0], se0[1], se0[2]);
		glTexCoord2d(1, 1);
		glVertex3d(ne0[0], ne0[1], ne0[2]);
		glTexCoord2d(0, 1);
		glVertex3d(ne1[0], ne1[1], ne1[2]);
	glEnd();
}

void cDrawUtil::DrawTriangle(const tVector& pos, double side_len, eDrawMode draw_mode)
{
	double h = std::sqrt(0.75 * side_len * side_len);
	tVector a = pos + tVector(0, 0.5 * h, 0, 0);
	tVector b = pos + tVector(-0.5 * side_len, -0.5 * h, 0, 0);
	tVector c = pos + tVector(0.5 * side_len, -0.5 * h, 0, 0);

	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLES : GL_LINE_LOOP;
	glTexCoord2d(0, 0);
	glBegin(gl_mode);
		glNormal3d(0, 0, 1);
		glVertex3d(a[0], a[1], a[2]);
		glVertex3d(b[0], b[1], b[2]);
		glVertex3d(c[0], c[1], c[2]);
	glEnd();
}

void cDrawUtil::DrawQuad(const tVector& a, const tVector& b, const tVector& c, const tVector& d, eDrawMode draw_mode)
{
	DrawQuad(a, b, c, d, tVector(0, 0, 0, 0), tVector(1, 0, 0, 0),
			tVector(1, 1, 0, 0), tVector(0, 1, 0, 0), draw_mode);
}

void cDrawUtil::DrawQuad(const tVector& a, const tVector& b, const tVector& c, const tVector& d,
						const tVector& coord_a, const tVector& coord_b, const tVector& coord_c, const tVector& coord_d, 
						eDrawMode draw_mode)
{
	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_QUADS : GL_LINE_LOOP;

	tVector normal = (b - a).cross3(d - a).normalized();
	glBegin(gl_mode);
		glNormal3d(normal[0], normal[1], normal[2]);

		glTexCoord2d(coord_a[0], coord_a[1]);
		glVertex3d(a[0], a[1], a[2]);

		glTexCoord2d(coord_b[0], coord_b[1]);
		glVertex3d(b[0], b[1], b[2]);

		glTexCoord2d(coord_c[0], coord_c[1]);
		glVertex3d(c[0], c[1], c[2]);

		glTexCoord2d(coord_d[0], coord_d[1]);
		glVertex3d(d[0], d[1], d[2]);
	glEnd();
}

void cDrawUtil::DrawDisk(const tVector& pos, double r, int slices, eDrawMode draw_mode)
{
	glPushMatrix();
	cDrawUtil::Translate(pos);
	DrawDisk(r, slices, draw_mode);
	glPopMatrix();
}

void cDrawUtil::DrawDisk(double r, int slices, eDrawMode draw_mode)
{
	double d_theta = 2.f * M_PI / (slices - 1);

	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLE_FAN : GL_LINE_LOOP;
	glBegin(gl_mode);

	glNormal3d(0, 0, 1);
	glTexCoord2d(0, 0);
	if (draw_mode == eDrawSolid)
	{
		glVertex3d(0.f, 0.f, 0.f);
	}
	
	for (int i = 0; i < slices; ++i)
	{
		double theta0 = i * d_theta;
		double theta1 = i * d_theta;

		double x0 = r * std::cos(theta0);
		double y0 = r * std::sin(theta0);
		double x1 = r * std::cos(theta1);
		double y1 = r * std::sin(theta1);

		glVertex3d(x0, y0, 0);
		glVertex3d(x1, y1, 0);
	}
	glEnd();
}

void cDrawUtil::DrawPoint(const tVector& pt)
{
	glBegin(GL_POINTS);
	glVertex3d(pt[0], pt[1], pt[2]);
	glEnd();
}

void cDrawUtil::DrawLine(const tVector& a, const tVector& b)
{
	glBegin(GL_LINES);
	glVertex3d(a[0], a[1], a[2]);
	glVertex3d(b[0], b[1], b[2]);
	glEnd();
}

void cDrawUtil::DrawLineStrip(const tVectorArr& pts)
{
	int num_pts = static_cast<int>(pts.size());
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < num_pts - 1; ++i)
	{
		const tVector& a = pts[i];
		const tVector& b = pts[i + 1];
		glVertex3d(a[0], a[1], a[2]);
		glVertex3d(b[0], b[1], b[2]);
	}
	glEnd();
}

void cDrawUtil::DrawStrip(const tVector& a, const tVector& b, double width, eDrawMode draw_mode)
{
	tVector delta = b - a;
	tVector orthogonal = tVector(-delta[1], delta[0], 0, 0);
	orthogonal.normalize();

	tVector v0 = a - width * 0.5 * orthogonal;
	tVector v1 = b - width * 0.5 * orthogonal;
	tVector v2 = b + width * 0.5 * orthogonal;
	tVector v3 = a + width * 0.5 * orthogonal;

	DrawQuad(v0, v1, v2, v3, draw_mode);
}

void cDrawUtil::DrawCross(const tVector& pos, double size)
{
	glBegin(GL_LINES);
	glVertex3d(pos[0] - 0.5 * size, pos[1], pos[2]);
	glVertex3d(pos[0] + 0.5 * size, pos[1], pos[2]);
	glVertex3d(pos[0], pos[1] - 0.5 * size, pos[2]);
	glVertex3d(pos[0], pos[1] + 0.5 * size, pos[2]);
	glEnd();
}

void cDrawUtil::DrawSphere(double r, int slices, int stacks, eDrawMode draw_mode)
{
	glPushMatrix();
	cDrawUtil::Rotate(M_PI / 2, tVector(1, 0, 0, 0));
	if (draw_mode == eDrawSolid)
	{
		glutSolidSphere(r, slices, stacks);
	}
	else
	{
		glutWireSphere(r, slices, stacks);
	}
	glPopMatrix();
}

void cDrawUtil::DrawCylinder(double h, double r, int slices, eDrawMode draw_mode)
{
	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLE_STRIP : GL_LINE_LOOP;

	glBegin(gl_mode);
	glTexCoord2d(0, 0);
	for (int i = 0; i <= slices; ++i)
	{
		double theta = i * 2 * M_PI / slices;
		double x = r * std::cos(theta);
		double z = r * std::sin(theta);

		tVector normal = tVector(x, z, 0, 0).normalized();

		glNormal3d(normal[0], normal[1], normal[2]);
		glVertex3d(x, -h * 0.5, z);
		glVertex3d(x, h * 0.5, z);
	}
	glEnd();
}

void cDrawUtil::DrawPlane(const tVector& coeffs, double size, eDrawMode draw_mode)
{
	const Eigen::Vector3d ref = Eigen::Vector3d(0, 0, 1);
	Eigen::Vector3d n = Eigen::Vector3d(coeffs[0], coeffs[1], coeffs[2]);
	double c = coeffs[3];

	Eigen::Vector3d axis = ref.cross(n);
	double axis_len = axis.norm();
	double theta = 0;
	if (axis_len != 0)
	{
		axis /= axis_len;
		theta = std::acos(ref.dot(n));
	}

	Eigen::Vector3d offset = c * n;

	glPushMatrix();
	cDrawUtil::Translate(tVector(offset[0], offset[1], offset[2], 0));
	if (theta != 0)
	{
		cDrawUtil::Rotate(theta, tVector(axis[0], axis[1], axis[2], 0));
	}
	DrawRect(tVector::Zero(), tVector(size, size, 0, 0), draw_mode);
	glPopMatrix();
}

void cDrawUtil::DrawCapsule(double h, double r, int slices, int stacks, eDrawMode draw_mode)
{
	glPushMatrix();
	DrawCylinder(h, r, slices, draw_mode);

	cDrawUtil::Translate(tVector(0, h * 0.5, 0, 0));
	DrawSphere(r, slices, stacks, draw_mode);
	cDrawUtil::Translate(tVector(0, -h, 0, 0));
	DrawSphere(r, slices, stacks, draw_mode);

	glPopMatrix();
}

void cDrawUtil::DrawArrow2D(const tVector& start, const tVector& end, double head_size)
{
	tVector dir = tVector(0, 1, 0, 0);
	double dir_len = 0;
	if (start != end)
	{
		dir = end - start;
		dir_len = dir.norm();
		dir /= dir_len;
	}
	
	dir[3] = 0;
	tVector axis = tVector(0, 0, 1, 0);
	tVector tangent = axis.cross3(dir);
	tangent.normalize();

	const double width = head_size * 0.1854;
	tVector body_end = end - dir * head_size;

	glBegin(GL_QUADS);

	tVector a = start - width * tangent;
	tVector b = body_end - width * tangent;
	tVector c = body_end + width * tangent;
	tVector d = start + width * tangent;

	glVertex3d(a[0], a[1], a[2]);
	glVertex3d(b[0], b[1], b[2]);
	glVertex3d(c[0], c[1], c[2]);
	glVertex3d(d[0], d[1], d[2]);

	glEnd();

	// draw arrow head

	glBegin(GL_TRIANGLES);

	tVector e0 = body_end - tangent * head_size * 0.5f;
	tVector e1 = body_end + tangent * head_size * 0.5f;

	glVertex3d(end[0], end[1], end[2]);
	glVertex3d(e0[0], e0[1], e0[2]);
	glVertex3d(e1[0], e1[1], e1[2]);

	glEnd();
}

void cDrawUtil::DrawGrid2D(const tVector& origin, const tVector& size, double spacing, double big_spacing)
{
	double w = size[0];
	double h = size[1];

	double min_x = origin(0) - w;
	double min_y = origin(1) - h;
	double max_x = origin(0) + w;
	double max_y = origin(1) + h;

	const double offset_z = origin[2];

	cDrawUtil::SetColor(tVector(188 / 255.f, 219 / 255.f, 242 / 255.f, 1.f));

	for (int i = 0; i < 2; ++i)
	{
		double curr_spacing = (i == 0) ? spacing : big_spacing;
		cDrawUtil::SetLineWidth((i == 0) ? 1 : 2);
		double strip_w = (i == 0) ? 0.005 : 0.015;

		for (double x = min_x - std::fmod(min_x, curr_spacing); x < max_x; x += curr_spacing)
		{
			tVector a = tVector(x, min_y, offset_z, offset_z);
			tVector b = tVector(x, max_y, offset_z, offset_z);
			cDrawUtil::DrawLine(a, b);
		}
		for (double y = min_y - std::fmod(min_y, curr_spacing); y < max_y; y += curr_spacing)
		{
			tVector a = tVector(min_x, y, offset_z, offset_z);
			tVector b = tVector(max_x, y, offset_z, offset_z);
			cDrawUtil::DrawLine(a, b);
		}
	}
}

void cDrawUtil::DrawRuler2D(const tVector& origin, const tVector& size, 
			const tVector& col, double marker_spacing, double big_marker_spacing,
			double marker_h, double big_marker_h)
{
	double w = size[0];
	double h = size[1];

	double min_x = origin(0) - w * 0.5;
	double min_y = origin(1) - h * 0.5;
	double max_x = origin(0) + w * 0.5;
	double max_y = origin(1) + h * 0.5;

	const double offset_z = origin[2];

	glTexCoord2d(0, 0);
	cDrawUtil::SetLineWidth(1);
	cDrawUtil::SetColor(col);
	cDrawUtil::DrawRect(origin, size, cDrawUtil::eDrawSolid);
	cDrawUtil::SetColor(tVector(0, 0, 0, 1));
	cDrawUtil::DrawLine(tVector(min_x, max_y, 0, 0), tVector(max_x, max_y, 0, 0));

	// draw markers
	cDrawUtil::SetColor(tVector(0.f, 0.f, 0.f, 1.f));

	for (int i = 0; i < 2; ++i)
	{
		bool big = (i == 1);
		cDrawUtil::SetLineWidth((big) ? 3 : 2);
		double curr_spacing = (big) ? big_marker_spacing : marker_spacing;
		double curr_h = (big) ? big_marker_h : marker_h;

		for (double x = min_x - std::fmod(min_x, curr_spacing); x < max_x; x += curr_spacing)
		{
			tVector a = tVector(x, max_y + curr_h * 0.5f, 0, 0);
			tVector b = tVector(x, max_y - curr_h * 0.5f, 0, 0);
			cDrawUtil::DrawLine(a, b);
		}
	}
}

void cDrawUtil::DrawSemiCircle(const tVector& pos, double r, int slices, 
							double min_theta, double max_theta, eDrawMode draw_mode)
{
	double d_theta = (max_theta - min_theta) / (slices - 1);

	GLenum gl_mode = (draw_mode == eDrawSolid) ? GL_TRIANGLE_FAN : GL_LINE_LOOP;
	glBegin(gl_mode);
	
	glTexCoord2d(0, 0);
	glNormal3d(0, 0, 1);
	glVertex3d(pos[0], pos[1], pos[2]);
	for (int i = 0; i < slices; ++i)
	{
		double theta0 = i * d_theta + min_theta;
		double theta1 = i * d_theta + min_theta;

		double x0 = r * std::cos(theta0) + pos[0];
		double y0 = r * std::sin(theta0) + pos[1];
		double x1 = r * std::cos(theta1) + pos[0];
		double y1 = r * std::sin(theta1) + pos[1];;

		glVertex3d(x0, y0, pos[2]);
		glVertex3d(x1, y1, pos[2]);
	}
	glEnd();
}

void cDrawUtil::DrawCalibMarker(const tVector& pos, double r, int slices, 
								const tVector& col0, const tVector& col1, 
								eDrawMode draw_mode)
{
	SetColor(col0);
	DrawSemiCircle(pos, r, slices, 0, M_PI * 0.5, draw_mode);
	DrawSemiCircle(pos, r, slices, M_PI, M_PI * 1.5, draw_mode);

	SetColor(col1);
	DrawSemiCircle(pos, r, slices, M_PI * 0.5, M_PI, draw_mode);
	DrawSemiCircle(pos, r, slices, M_PI * 1.5, M_PI * 2, draw_mode);
}

void cDrawUtil::DrawString(const std::string& str, const tVector& scale)
{
	std::istringstream str_stream(str);
	std::string curr_str = "";

	glPushMatrix();
	const double default_scale = 0.0045f;
	cDrawUtil::Scale(tVector(default_scale * scale[0], default_scale * scale[1], scale[2], 0));

	while (std::getline(str_stream, curr_str))
	{
		glPushMatrix();
		for (size_t i = 0; i < curr_str.size(); ++i)
		{
			glutStrokeCharacter(GLUT_STROKE_MONO_ROMAN, curr_str[i]);
		}
		glPopMatrix();

		cDrawUtil::Translate(tVector(0, -1 / default_scale, 0, 0));
	}
	glPopMatrix();
}

void cDrawUtil::DrawTexQuad(const cTextureDesc& tex, const tVector& pos, const tVector& size)
{
	tex.BindTex(GL_TEXTURE0);
	DrawQuad(tVector(pos[0] - 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2], 0),
				tVector(pos[0] + 0.5 * size[0], pos[1] - 0.5 * size[1], pos[2], 0),
				tVector(pos[0] + 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2], 0),
				tVector(pos[0] - 0.5 * size[0], pos[1] + 0.5 * size[1], pos[2], 0),
				tVector(0, 0, 0, 0), tVector(1, 0, 0, 0), tVector(1, 1, 0, 0), tVector(0, 1, 0, 0),
				eDrawSolid);
	tex.UnbindTex(GL_TEXTURE0);
}

void cDrawUtil::CopyTexture(const cTextureDesc& src_tex, const cTextureDesc& dst_tex)
{
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	GLint depth_test_param;
	glGetIntegerv(GL_DEPTH_TEST, &depth_test_param);
	glDisable(GL_DEPTH_TEST);

	GLint blend_param;
	glGetIntegerv(GL_BLEND, &blend_param);
	glDisable(GL_BLEND);

	SetColor(tVector::Ones());
	dst_tex.BindBuffer();
	DrawTexQuad(src_tex, tVector::Zero(), tVector(2, 2, 0, 0));
	dst_tex.UnbindBuffer();

	if (depth_test_param == 1)
	{
		glEnable(GL_DEPTH_TEST);
	}

	if (blend_param == 1)
	{
		glEnable(GL_BLEND);
	}

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
}

void cDrawUtil::ClearColor(const tVector& col)
{
	glClearColor(static_cast<float>(col[0]), static_cast<float>(col[1]),
				static_cast<float>(col[2]), static_cast<float>(col[3]));
	glClear(GL_COLOR_BUFFER_BIT);
}

void cDrawUtil::ClearDepth(double depth)
{
	glClearDepth(depth);
	glClear(GL_DEPTH_BUFFER_BIT);
}

void cDrawUtil::GLMultMatrix(const tMatrix& mat)
{
	glMultMatrixd(mat.data());
}

void cDrawUtil::Finish()
{
	glFinish();
}

void cDrawUtil::Translate(const tVector& trans)
{
	glTranslated(trans[0], trans[1], trans[2]);
}

void cDrawUtil::Scale(const tVector& scale)
{
	glScaled(scale[0], scale[1], scale[2]);
}

void cDrawUtil::Rotate(double theta, const tVector& axis)
{
	glRotated(theta * gRadiansToDegrees, axis[0], axis[1], axis[2]);
}

void cDrawUtil::SetColor(const tVector& col)
{
	glColor4d(col[0], col[1], col[2], col[3]);
}

void cDrawUtil::SetLineWidth(double w)
{
	glLineWidth(static_cast<float>(w));
}

void cDrawUtil::SetPointSize(double pt_size)
{
	glPointSize(static_cast<float>(pt_size));
}
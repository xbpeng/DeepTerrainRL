#pragma once

#include "anim/Character.h"

class cDrawCharacter
{
public:
	static void Draw(const cCharacter& character, double link_width, const tVector& fill_col, const tVector& line_col);
	static void DrawShape(const cCharacter& character, const cKinTree::tDrawShapeDef& def, const tVector& fill_tint, const tVector& line_col);

protected:
	static void DrawShapeBox(const cCharacter& character, const cKinTree::tDrawShapeDef& def, const tVector& fill_tint, const tVector& line_col);
	static void DrawShapeCapsule(const cCharacter& character, const cKinTree::tDrawShapeDef& def, const tVector& fill_tint, const tVector& line_col);
};

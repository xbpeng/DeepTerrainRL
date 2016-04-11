#include "DrawCharacter.h"
#include "DrawKinTree.h"
#include "render/DrawUtil.h"
#include <iostream>

void cDrawCharacter::Draw(const cCharacter& character, double link_width, const tVector& fill_col, const tVector& line_col)
{
	const Eigen::MatrixXd& joint_mat = character.GetJointMat();
	Eigen::VectorXd pose;
	character.BuildPose(pose);
	cDrawKinTree::Draw(joint_mat, pose, link_width, fill_col, line_col);
}

void cDrawCharacter::DrawShape(const cCharacter& character, const cKinTree::tDrawShapeDef& def, const tVector& fill_tint, const tVector& line_col)
{
	cKinTree::eBodyShape shape = static_cast<cKinTree::eBodyShape>((int) def[cKinTree::eDrawShapeShape]);
	switch (shape)
	{
	case cKinTree::eBodyShapeBox:
		DrawShapeBox(character, def, fill_tint, line_col);
		break;
	case cKinTree::eBodyShapeCapsule:
		DrawShapeCapsule(character, def, fill_tint, line_col);
		break;
	case cKinTree::eBodyShapeNULL:
		break;
	default:
		assert(false); // unsupported draw shape
		break;
	}
}

void cDrawCharacter::DrawShapeBox(const cCharacter& character, const cKinTree::tDrawShapeDef& def, const tVector& fill_tint, const tVector& line_col)
{
	double theta = 0;
	tVector axis = tVector(0, 0, 1, 0);
	cKinTree::GetDrawShapeRotation(def, axis, theta);
	int parent_joint = cKinTree::GetDrawShapeParentJoint(def);
	tVector attach_pt = cKinTree::GetDrawShapeAttachPt(def);
	tVector col = cKinTree::GetDrawShapeColor(def);
	tVector size = tVector(def[cKinTree::eDrawShapeParam0], def[cKinTree::eDrawShapeParam1], def[cKinTree::eDrawShapeParam2], 0);
	col = col.cwiseProduct(fill_tint);

	tMatrix world_trans = character.BuildJointWorldTrans(parent_joint);
	
	glPushMatrix();
	cDrawUtil::GLMultMatrix(world_trans);
	cDrawUtil::Translate(attach_pt);
	cDrawUtil::Rotate(theta, axis);

	cDrawUtil::SetColor(col);
	cDrawUtil::DrawBox(tVector::Zero(), size, cDrawUtil::eDrawSolid);

	if (line_col[3] > 0)
	{
		cDrawUtil::SetColor(line_col);
		cDrawUtil::DrawBox(tVector::Zero(), size, cDrawUtil::eDrawWire);
	}

	glPopMatrix();
}

void cDrawCharacter::DrawShapeCapsule(const cCharacter& character, const cKinTree::tDrawShapeDef& def, const tVector& fill_tint, const tVector& line_col)
{
	double theta = 0;
	tVector axis = tVector(0, 0, 1, 0);
	cKinTree::GetDrawShapeRotation(def, axis, theta);
	int parent_joint = cKinTree::GetDrawShapeParentJoint(def);
	tVector attach_pt = cKinTree::GetDrawShapeAttachPt(def);
	tVector col = cKinTree::GetDrawShapeColor(def);
	tVector size = tVector(def[cKinTree::eDrawShapeParam0], def[cKinTree::eDrawShapeParam1], 0, 0);
	col = col.cwiseProduct(fill_tint);

	tMatrix world_trans = character.BuildJointWorldTrans(parent_joint);

	glPushMatrix();
	cDrawUtil::GLMultMatrix(world_trans);
	cDrawUtil::Translate(attach_pt);
	cDrawUtil::Rotate(theta, axis);

	cDrawUtil::SetColor(col);
	cDrawUtil::DrawCapsule(size[0], size[1], 8, 8, cDrawUtil::eDrawSolid);

	if (line_col[3] > 0)
	{
		cDrawUtil::SetColor(line_col);
		cDrawUtil::DrawCapsule(size[0], size[1], 8, 8, cDrawUtil::eDrawWire);
	}

	glPopMatrix();
}
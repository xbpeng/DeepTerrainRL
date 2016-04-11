#include "DrawKinTree.h"

#include "render/DrawUtil.h"

void cDrawKinTree::Draw(const Eigen::MatrixXd& joint_desc, const Eigen::VectorXd& pose, double link_width, const tVector& fill_col, const tVector& line_col)
{
	int root_id = cKinTree::GetRoot(joint_desc);
	cDrawUtil::SetLineWidth(1);
	DrawTree(joint_desc, pose, root_id, link_width, fill_col, line_col);
}

void cDrawKinTree::DrawTree(const Eigen::MatrixXd& joint_desc, const Eigen::VectorXd& pose, int joint_id, double link_width,
						const tVector& fill_col, const tVector& line_col)
{
	const double node_radius = 0.02;

	if (joint_id != cKinTree::gInvalidJointID)
	{
		bool has_parent = cKinTree::HasParent(joint_desc, joint_id);
		if (has_parent)
		{
			tVector attach_pt = cKinTree::GetScaledAttachPt(joint_desc, joint_id);
			attach_pt[2] = 0; // hack ignore z
			double len = attach_pt.norm();

			glPushMatrix();
			tVector pos = tVector(len / 2, 0, 0, 0);
			tVector size = tVector(len, link_width, 0, 0);

			double theta = std::atan2(attach_pt[1], attach_pt[0]);
			cDrawUtil::Rotate(theta, tVector(0, 0, 1, 0));

			cDrawUtil::SetColor(tVector(fill_col[0], fill_col[1], fill_col[2], fill_col[3]));
			cDrawUtil::DrawRect(pos, size, cDrawUtil::eDrawSolid);
			cDrawUtil::SetColor(tVector(line_col[0], line_col[1], line_col[2], line_col[3]));
			cDrawUtil::DrawRect(pos, size, cDrawUtil::eDrawWire);
			
			// draw node
			cDrawUtil::SetColor(tVector(fill_col[0] * 0.25, fill_col[1] * 0.25, fill_col[2] * 0.25, fill_col[3]));
			cDrawUtil::DrawDisk(node_radius, 16);
			
			glPopMatrix();
		}

		glPushMatrix();
		tMatrix m = cKinTree::ChildParentTrans(joint_desc, pose, joint_id);
		cDrawUtil::GLMultMatrix(m);

		Eigen::VectorXi children;
		cKinTree::FindChildren(joint_desc, joint_id, children);
		for (int i = 0; i < children.size(); ++i)
		{
			int child_id = children[i];
			DrawTree(joint_desc, pose, child_id, link_width, fill_col, line_col);
		}

		glPopMatrix();
	}
}
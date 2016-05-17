#include "KinTree.h"

#include <iostream>

#include "util/FileUtil.h"
#include "sim/RBDUtil.h"

const int cKinTree::gPosDims = 2;
const int cKinTree::gInvalidJointID = -1;

// Json keys
const std::string gJointsKey = "Joints";
const std::string gJointDescKeys[cKinTree::eJointDescMax] = 
{
	"Type",
	"Parent",
	"AttachX",
	"AttachY",
	"AttachZ",
	"Scale",
	"LinkStiffness",
	"LimLow",
	"LimHigh",
	"Offset"
};
const std::string gBodyDefsKey = "BodyDefs";
const std::string gBodyDescKeys[cKinTree::eBodyParamMax] =
{
	"Shape",
	"Mass",
	"AttachX",
	"AttachY",
	"AttachZ",
	"Theta",
	"Param0",
	"Param1",
	"Param2",
	"ColorR",
	"ColorG",
	"ColorB",
	"ColorA",
};

const std::string gDrawShapeDefsKey = "DrawShapeDefs";
const std::string gDrawShapeDescKeys[cKinTree::eDrawShapeParamMax] =
{
	"Shape",
	"ParentJoint",
	"AttachX",
	"AttachY",
	"AttachZ",
	"Theta",
	"Param0",
	"Param1",
	"Param2",
	"ColorR",
	"ColorG",
	"ColorB",
	"ColorA",
};

int cKinTree::GetRoot(const Eigen::MatrixXd& joint_desc)
{
	// this should always be true right?
	return 0;
	/*
	int num_joints = GetNumJoints(joint_desc);
	for (int i = 0; i < num_joints; ++i)
	{
		int parent = GetParent(joint_desc, i);
		if (parent == gInvalidJointID)
		{
			return i;
		}
	}
	assert(false);
	return gInvalidJointID;
	*/
}

void cKinTree::FindChildren(const Eigen::MatrixXd& joint_desc, int joint_id, Eigen::VectorXi& out_children)
{
	const int max_size = 128;
	int children_buffer[max_size];
	int num_children = 0;
	int num_joints = GetNumJoints(joint_desc);

	for (int i = 0; i < num_joints; ++i)
	{
		int parent = GetParent(joint_desc, i);
		if (parent == joint_id)
		{
			children_buffer[num_children] = i;
			++num_children;

			if (num_children >= max_size)
			{
				printf("Too many children, max = %i", max_size);
				assert(false);
				return;
			}
		}
	}

	out_children.resize(num_children);
	for (int i = 0; i < num_children; ++i)
	{
		out_children[i] = children_buffer[i];
	}
}

bool cKinTree::LoadBodyDefs(const std::string& char_file, Eigen::MatrixXd& out_body_defs)
{
	bool succ = true;
	std::string str;
	
	std::ifstream f_stream(char_file.c_str());
	Json::Value root;
	Json::Reader reader;
	succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ)
	{
		succ = false;
		if (!root[gBodyDefsKey].isNull())
		{
			Json::Value body_defs = root.get(gBodyDefsKey, 0);
			int num_bodies = body_defs.size();

			succ = true;
			out_body_defs.resize(num_bodies, eBodyParamMax);
			for (int b = 0; b < num_bodies; ++b)
			{
				cKinTree::tBodyDef curr_def;
				Json::Value body_json = body_defs.get(b, 0);
				bool succ_def = ParseBodyDef(body_json, curr_def);

				if (succ)
				{
					out_body_defs.row(b) = curr_def;
				}
				else
				{
					succ = false;
					break;
				}
			}
		}
	}
	
	if (!succ)
	{
		printf("Failed to load body definition from %s\n", char_file.c_str());
		succ = false;
	}

	return succ;
}

bool cKinTree::ParseBodyDef(const Json::Value& root, cKinTree::tBodyDef& out_def)
{
	out_def.setZero();

	std::string shape_str = root.get(gBodyDescKeys[eBodyParamShape], "").asString();
	eBodyShape shape;
	bool succ = ParseBodyShape(shape_str, shape);
	if (succ)
	{
		out_def(eBodyParamShape) = static_cast<double>(static_cast<int>(shape));
	}

	for (int i = 0; i < eBodyParamMax; ++i)
	{
		const std::string& curr_key = gBodyDescKeys[i];
		if (!root[curr_key].isNull()
			&& root[curr_key].isNumeric())
		{
			Json::Value json_val = root[curr_key];
			double val = json_val.asDouble();
			out_def(i) = val;
		}
	}

	return succ;
}

bool cKinTree::ParseBodyShape(const std::string& str, cKinTree::eBodyShape& out_shape)
{
	bool succ = true;
	if (str == "box")
	{
		out_shape = eBodyShapeBox;
	}
	else if (str == "capsule")
	{
		out_shape = eBodyShapeCapsule;
	}
	else if (str == "null")
	{
		out_shape = eBodyShapeNULL;
	}
	else
	{
		printf("Unsupported body shape %s\n", str.c_str());
		assert(false);
	}
	return succ;
}

bool cKinTree::LoadDrawShapeDefs(const std::string& char_file, Eigen::MatrixXd& out_draw_defs)
{
	bool succ = true;
	std::string str;

	std::ifstream f_stream(char_file.c_str());
	Json::Value root;
	Json::Reader reader;
	succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ)
	{
		if (!root[gDrawShapeDefsKey].isNull())
		{
			Json::Value shape_defs = root.get(gDrawShapeDefsKey, 0);
			int num_shapes = shape_defs.size();

			succ = true;
			out_draw_defs.resize(num_shapes, eDrawShapeParamMax);
			for (int b = 0; b < num_shapes; ++b)
			{
				cKinTree::tDrawShapeDef curr_def;
				Json::Value shape_json = shape_defs.get(b, 0);
				bool succ_def = ParseDrawShapeDef(shape_json, curr_def);

				if (succ)
				{
					out_draw_defs.row(b) = curr_def;
				}
				else
				{
					succ = false;
					break;
				}
			}
		}
	}

	if (!succ)
	{
		printf("Failed to load draw shape definition from %s\n", char_file.c_str());
		assert(false);
	}

	return succ;
}

bool cKinTree::ParseDrawShapeDef(const Json::Value& root, tBodyDef& out_def)
{
	out_def.setZero();

	std::string shape_str = root.get(gDrawShapeDescKeys[eDrawShapeShape], "").asString();
	eBodyShape shape;
	bool succ = ParseBodyShape(shape_str, shape);
	if (succ)
	{
		out_def(eDrawShapeShape) = static_cast<double>(static_cast<int>(shape));
	}

	for (int i = 0; i < eDrawShapeParamMax; ++i)
	{
		const std::string& curr_key = gDrawShapeDescKeys[i];
		if (!root[curr_key].isNull()
			&& root[curr_key].isNumeric())
		{
			Json::Value json_val = root[curr_key];
			double val = json_val.asDouble();
			out_def(i) = val;
		}
	}

	return succ;
}

tVector cKinTree::CalcBodyPartPos(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::MatrixXd& body_defs, int part_id)
{
	assert(IsValidBody(body_defs, part_id));
	tMatrix body_joint_trans = BodyJointTrans(body_defs, part_id);
	tMatrix joint_to_world_trans = JointWorldTrans(joint_mat, state, part_id);
	
	tVector attach_pt = tVector(0, 0, 0, 1);
	attach_pt = joint_to_world_trans * (body_joint_trans * attach_pt);
	return attach_pt;
}

cKinTree::eBodyShape cKinTree::GetBodyShape(const Eigen::MatrixXd& body_defs, int part_id)
{
	cKinTree::eBodyShape shape = static_cast<cKinTree::eBodyShape>(static_cast<int>(body_defs(part_id, cKinTree::eBodyParamShape)));
	return shape;
}

tVector cKinTree::GetBodyAttachPt(const Eigen::MatrixXd& body_defs, int part_id)
{
	const cKinTree::tBodyDef& def = body_defs.row(part_id);
	tVector attach_pt = tVector(def(eBodyParamAttachX), def(eBodyParamAttachY), def(eBodyParamAttachZ), 0);
	return attach_pt;
}

void cKinTree::GetBodyRotation(const Eigen::MatrixXd& body_defs, int part_id, tVector& out_axis, double& out_theta)
{
	const cKinTree::tBodyDef& def = body_defs.row(part_id);
	out_theta = def(eBodyParamTheta);
	out_axis = tVector(0, 0, 1, 0);
}

double cKinTree::GetBodyMass(const Eigen::MatrixXd& body_defs, int part_id)
{
	double mass = body_defs(part_id, cKinTree::eBodyParamMass);
	return mass;
}

tVector cKinTree::GetBodySize(const Eigen::MatrixXd& body_defs, int part_id)
{
	const tBodyDef& def = body_defs.row(part_id);
	tVector size = tVector(def(eBodyParam0), def(eBodyParam1), def(eBodyParam2), 0);
	return size;
}

tVector cKinTree::GetBodyColor(const Eigen::MatrixXd& body_defs, int part_id)
{
	const tBodyDef& def = body_defs.row(part_id);
	tVector col = tVector(def(eBodyColorR), def(eBodyColorG), def(eBodyColorB), def(eBodyColorA));
	return col;
}

double cKinTree::CalcTotalMass(const Eigen::MatrixXd& body_defs)
{
	double total_mass = 0;
	for (int i = 0; i < body_defs.rows(); ++i)
	{
		if (IsValidBody(body_defs, i))
		{
			double mass = cKinTree::GetBodyMass(body_defs, i);
			total_mass += mass;
		}
	}
	return total_mass;
}

bool cKinTree::IsValidBody(const Eigen::MatrixXd& body_defs, int part_id)
{
	eBodyShape shape = GetBodyShape(body_defs, part_id);
	if (shape < eBodyShapeMax)
	{
		return true;
	}
	return false;
}

tVector cKinTree::GetBodyLocalCoM(const Eigen::MatrixXd& body_defs, int part_id)
{
	eBodyShape shape = GetBodyShape(body_defs, part_id);
	tVector com = tVector::Zero();
	switch (shape)
	{
	case eBodyShapeBox:
		com.setZero();
		break;
	case eBodyShapeCapsule:
		com.setZero();
		break;
	default:
		assert(false); // unsupported
		break;
	}

	return com;
}

int cKinTree::GetDrawShapeParentJoint(const tDrawShapeDef& shape)
{
	return static_cast<int>(shape[eDrawShapeParentJoint]);
}

tVector cKinTree::GetDrawShapeAttachPt(const tDrawShapeDef& shape)
{
	return tVector(shape[cKinTree::eDrawShapeAttachX], shape[cKinTree::eDrawShapeAttachY], shape[cKinTree::eDrawShapeAttachZ], 0);
}

void cKinTree::GetDrawShapeRotation(const tDrawShapeDef& shape, tVector& out_axis, double& out_theta)
{
	out_theta = shape[cKinTree::eDrawShapeTheta];
	out_axis = tVector(0, 0, 1, 0);
}

tVector cKinTree::GetDrawShapeColor(const tDrawShapeDef& shape)
{
	return tVector(shape[cKinTree::eDrawShapeColorR], shape[cKinTree::eDrawShapeColorG], shape[cKinTree::eDrawShapeColorB], shape[cKinTree::eDrawShapeColorA]);
}


void cKinTree::CalcBodyPartRotation(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::MatrixXd& body_defs, int part_id, tVector& out_axis, double& out_theta)
{
	tMatrix mat = BodyWorldTrans(joint_mat, state, body_defs, part_id);
	cMathUtil::RotMatToAxisAngle(mat, out_axis, out_theta);
}

bool cKinTree::Load(const Json::Value& root, Eigen::MatrixXd& out_joint_mat)
{
	bool succ = false;

	if (!root[gJointsKey].isNull())
	{
		Json::Value joints = root[gJointsKey];
		int num_joints = joints.size();

		out_joint_mat.resize(num_joints, eJointDescMax);
		
		for (int j = 0; j < num_joints; ++j)
		{
			tJointDesc curr_joint_desc = tJointDesc::Zero();

			Json::Value joint_json = joints.get(j, 0);
			succ = ParseJoint(joint_json, curr_joint_desc);
			if (succ)
			{
				out_joint_mat.row(j) = curr_joint_desc;
			}
			else
			{
				printf("Failed to parse joint %i\n", j);
				return false;
			}
		}

		for (int j = 0; j < num_joints; ++j)
		{
			const auto& curr_desc = out_joint_mat.row(j);
			int parent_id = static_cast<int>(curr_desc(eJointDescParent));
			if (parent_id >= j)
			{
				printf("Parent id must be < child id, parent id: %i, child id: %i\n", parent_id, j);
				out_joint_mat.resize(0, 0);
				assert(false);

				return false;
			}

			out_joint_mat.row(j) = curr_desc;
		}

		PostProcessJointMat(out_joint_mat);
	}

	return succ;
}

bool cKinTree::HasValidRoot(const Eigen::MatrixXd& joint_desc)
{
	int root = GetRoot(joint_desc);
	return root != gInvalidJointID;
}

tVector cKinTree::GetRootPos(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state)
{
	int root = GetRoot(joint_mat);
	tVector pos = GetJointOffset(joint_mat, state, root);
	return pos;
}

void cKinTree::SetRootPos(const Eigen::MatrixXd& joint_mat, const tVector& pos, Eigen::VectorXd& out_state)
{
	int root = GetRoot(joint_mat);
	SetJointOffset(joint_mat, root, pos, out_state);
}

void cKinTree::SetRootPosX(const Eigen::MatrixXd& joint_mat,  double x, Eigen::VectorXd& out_state)
{
	tVector pos = GetRootPos(joint_mat, out_state);
	pos[0] = x;
	SetRootPos(joint_mat, pos, out_state);
}


double cKinTree::GetRootTheta(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state)
{
	int root_id = GetRoot(joint_mat);
	double theta = GetJointTheta(joint_mat, state, root_id);
	return theta;
}

void cKinTree::SetRootTheta(const Eigen::MatrixXd& joint_mat,  double theta, Eigen::VectorXd& out_state)
{
	int root = GetRoot(joint_mat);
	SetJointTheta(joint_mat, root, theta, out_state);
}

tVector cKinTree::CalcJointWorldPos(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	tVector pos = LocalToWorldPos(joint_mat, state, joint_id, tVector::Zero());
	return pos;
}

tVector cKinTree::CalcJointWorldVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::VectorXd& vel, int joint_id)
{
	return CalcWorldVel(joint_mat, state, vel, joint_id, tVector::Zero());
}

tVector cKinTree::CalcWorldVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::VectorXd& vel, int parent_id, const tVector& attach_pt)
{
	cSpAlg::tSpVec sv = cRBDUtil::CalcWorldVel(joint_mat, state, vel, parent_id, attach_pt);
	tVector pos = cKinTree::LocalToWorldPos(joint_mat, state, parent_id, attach_pt);
	cSpAlg::tSpTrans world_to_pt = cSpAlg::BuildTrans(pos);
	sv = cSpAlg::ApplyTransM(world_to_pt, sv);
	return cSpAlg::GetV(sv);
}

tVector cKinTree::LocalToWorldPos(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int parent_id, const tVector& attach_pt)
{
	tMatrix local_to_world_trans = JointWorldTrans(joint_mat, state, parent_id);
	tVector pos = attach_pt;
	pos[3] = 1;
	pos = local_to_world_trans * pos;
	pos[3] = 0;

	return pos;
}

void cKinTree::CalcJointWorldTheta(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id,
									tVector& out_axis, double& out_theta)
{
	int parent_id = GetParent(joint_mat, joint_id);
	double theta = GetJointTheta(joint_mat, state, joint_id);
	out_theta = LocalToWorldTheta(joint_mat, state, parent_id, theta);
	out_axis = tVector(0, 0, 1, 0);
}

double cKinTree::LocalToWorldTheta(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int parent_id, double local_theta)
{
	// assumes 2d and joint axis along z
	int curr_id = parent_id;
	double curr_theta = local_theta;

	while (curr_id != gInvalidJointID)
	{
		double theta = GetJointTheta(joint_mat, state, curr_id);
		curr_theta += theta;
		curr_id = GetParent(joint_mat, curr_id);
	}

	return curr_theta;
}

int cKinTree::GetNumJoints(const Eigen::MatrixXd& joint_mat)
{
	return static_cast<int>(joint_mat.rows());
}

int cKinTree::GetNumDof(const Eigen::MatrixXd& joint_mat)
{
	int num_joints = GetNumJoints(joint_mat);
	int num_dof = cKinTree::GetParamOffset(joint_mat, num_joints - 1) + cKinTree::GetParamSize(joint_mat, num_joints - 1);
#if !defined(DISABLE_LINK_SCALE)
	num_dof += num_joints;
#endif
	return num_dof;
}

void cKinTree::ApplyStep(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& step, Eigen::VectorXd& out_pose)
{
	int root_id = GetRoot(joint_mat);
	int num_joints = cKinTree::GetNumJoints(joint_mat);
	out_pose += step;

#if !defined(DISABLE_LINK_SCALE)
	int idx_offset = gPosDims + num_joints;
	out_joint_mat.col(eJointDescScale) += step.block(idx_offset, 0, num_joints, 1);
	// no scaling for root
	out_pose(idx_offset + root_id) = 1;
#endif
}


Eigen::VectorXi cKinTree::FindJointChain(const Eigen::MatrixXd& joint_mat, int joint_beg, int joint_end)
{
	Eigen::VectorXi chain;

	const int max_length = 128;
	int chain_buffer[max_length];
	int buffer_idx = 0;

	if (joint_beg == joint_end)
	{
		Eigen::VectorXi chain(1);
		chain[0] = joint_beg;
	}

	int common_ancestor = gInvalidJointID;
	int curr_id = joint_beg;
	int end_len = 0;
	while (curr_id != gInvalidJointID)
	{
		chain_buffer[buffer_idx] = curr_id;
		++buffer_idx;

		if (buffer_idx >= max_length)
		{
			printf("Exceeded maximum chain length %i\n", max_length);
			assert(false);
			return chain;
		}

		bool is_ancestor = IsAncestor(joint_mat, joint_end, curr_id, end_len);
		if (is_ancestor)
		{
			common_ancestor = curr_id;
			break;
		}
		else
		{
			curr_id = GetParent(joint_mat, curr_id);
		}
	}

	bool found = common_ancestor != gInvalidJointID;
	// tree should always connected?
	assert(found);

	if (found)
	{
		chain.resize(buffer_idx + end_len);
		for (int i = 0; i < buffer_idx; ++i)
		{
			chain[i] = chain_buffer[i];
		}

		int idx = buffer_idx;
		curr_id = joint_end;
		while (curr_id != common_ancestor)
		{
			chain[idx] = curr_id;
			curr_id = GetParent(joint_mat, curr_id);
			++idx;
		}

		int num_flips = static_cast<int>(chain.size()) - buffer_idx;
		chain.block(buffer_idx, 0, num_flips, 1).reverseInPlace();
	}

	return chain;
}

bool cKinTree::IsAncestor(const Eigen::MatrixXd& joint_mat, int child_joint, int ancestor_joint, int& out_len)
{
	int curr_id = child_joint;
	out_len = 0;
	while (curr_id != gInvalidJointID)
	{
		if (curr_id == ancestor_joint)
		{
			return true;
		}
		else
		{
			curr_id = GetParent(joint_mat, curr_id);
			++out_len;
		}
	}
	return false;
}

double cKinTree::CalcChainLength(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXi& chain)
{
	double len = 0;
	int num_joints = static_cast<int>(chain.size());
	for (int i = 1; i < num_joints; ++i)
	{
		int curr_id = chain(i);
		int prev_id = chain(i - 1);

		if (prev_id != gInvalidJointID)
		{
			int prev_parent = GetParent(joint_mat, prev_id);
			bool is_parent = (prev_parent == curr_id);
			if (is_parent)
			{
				double curr_len = CalcLinkLength(joint_mat, prev_id);
				len += curr_len;
			}
		}

		if (curr_id != gInvalidJointID)
		{
			int curr_parent = GetParent(joint_mat, curr_id);
			bool is_child = (curr_parent == prev_id);
			if (is_child)
			{
				double curr_len = CalcLinkLength(joint_mat, curr_id);
				len += curr_len;
			}
		}
	}

	return len;
}

void cKinTree::CalcAABB(const Eigen::MatrixXd& joint_desc, const Eigen::VectorXd& state, tVector& out_min, tVector& out_max)
{
	out_min[0] = std::numeric_limits<double>::infinity();
	out_min[1] = std::numeric_limits<double>::infinity();
	out_min[2] = std::numeric_limits<double>::infinity();

	out_max[0] = -std::numeric_limits<double>::infinity();
	out_max[1] = -std::numeric_limits<double>::infinity();
	out_max[2] = -std::numeric_limits<double>::infinity();

	for (int i = 0; i < GetNumJoints(joint_desc); ++i)
	{
		tVector pos = CalcJointWorldPos(joint_desc, state, i);
		out_min = out_min.cwiseMin(pos);
		out_max = out_max.cwiseMax(pos);
	}
}

int cKinTree::GetParamOffset(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	int offset = static_cast<int>(joint_mat(joint_id, eJointDescParamOffset));
	return offset;
}

int cKinTree::GetParamSize(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	int size = 0;
	eJointType j_type = cKinTree::GetJointType(joint_mat, joint_id);
	bool is_root = cKinTree::IsRoot(joint_mat, joint_id);
	switch (j_type)
	{
	case eJointTypeRevolute:
		size = 1;
		break;
	case eJointTypePrismatic:
		size = 1;
		break;
	case eJointTypePlanar:
		size = 3;
		break;
	case eJointTypeFixed:
		size = (is_root) ? 3 : 0;
		break;
	default:
		break;
	}


	return size;
}

Eigen::VectorXd cKinTree::GetJointParams(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int j)
{
	int offset = cKinTree::GetParamOffset(joint_mat, j);
	int dim = cKinTree::GetParamSize(joint_mat, j);
	Eigen::VectorXd q;
	if (dim > 0)
	{
		q = state.block(offset, 0, dim, 1);
	}
	else
	{
		q = Eigen::VectorXd::Zero(1);
	}
	return q;
}

void cKinTree::SetJointParams(const Eigen::MatrixXd& joint_mat, int j, const Eigen::VectorXd& params, Eigen::VectorXd& out_state)
{
	int offset = cKinTree::GetParamOffset(joint_mat, j);
	int dim = cKinTree::GetParamSize(joint_mat, j);
	assert(dim == params.size());
	out_state.block(offset, 0, dim, 1) = params;
}

cKinTree::eJointType cKinTree::GetJointType(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	eJointType type = static_cast<eJointType>(static_cast<int>(joint_mat(joint_id, cKinTree::eJointDescType)));
	return type;
}

int cKinTree::GetParent(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	int parent = static_cast<int>(joint_mat(joint_id, cKinTree::eJointDescParent));
	assert(parent < joint_id); // joints should always be ordered as such
								// since some algorithms will assume this ordering
	return parent;
}

bool cKinTree::HasParent(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	int parent = GetParent(joint_mat, joint_id);
	return parent != gInvalidJointID;
}

bool cKinTree::IsRoot(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	return !HasParent(joint_mat, joint_id);
}

bool cKinTree::IsJointActuated(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	return !IsRoot(joint_mat, joint_id);
}

tVector cKinTree::GetJointOffset(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	tVector offset = tVector::Zero();
	eJointType j_type = GetJointType(joint_mat, joint_id);
	bool is_root = cKinTree::IsRoot(joint_mat, joint_id);

	int param_offset = GetParamOffset(joint_mat, joint_id);
	switch (j_type)
	{
	case eJointTypeRevolute:
		offset.setZero();
		break;
	case eJointTypePlanar:
		offset.segment(0, 2) = state.segment(param_offset, 2);
		break;
	case eJointTypePrismatic:
		offset.segment(0, 1) = state.segment(param_offset, 0);
		break;
	case eJointTypeFixed:
		offset.segment(0, 2) = (is_root) ? state.segment(param_offset, 2) : Eigen::Vector2d(0, 0);
		break;
	default:
		assert(false); // unsupported joint
		break;
	}
	
	return offset;
}

void cKinTree::SetJointOffset(const Eigen::MatrixXd& joint_mat, int joint_id, const tVector& offset, Eigen::VectorXd& out_state)
{
	eJointType j_type = GetJointType(joint_mat, joint_id);
	int param_offset = GetParamOffset(joint_mat, joint_id);
	switch (j_type)
	{
	case eJointTypePrismatic:
		out_state.segment(param_offset, 1) = offset.segment(0, 1);
		break;
	case eJointTypePlanar:
		out_state.segment(param_offset, 2) = offset.segment(0, 2);
		break;
	case eJointTypeFixed:
		// TODO not the best fix
		out_state.segment(param_offset, 2) = offset.segment(0, 2);
		break;
	default:
		assert(false && "No offset for this joint type"); // no offset for joint type
		break;
	}
}


double cKinTree::GetJointTheta(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	double theta = 0;
	eJointType j_type = GetJointType(joint_mat, joint_id);
	int param_offset = GetParamOffset(joint_mat, joint_id);
	Eigen::VectorXd params = GetJointParams(joint_mat, state, joint_id);
	bool is_root = cKinTree::IsRoot(joint_mat, joint_id);

	switch (j_type)
	{
	case eJointTypeRevolute:
		theta = state(param_offset);
		break;
	case eJointTypePrismatic:
		theta = state(param_offset);
		break;
	case eJointTypePlanar:
		theta = state(param_offset + 2);
		break;
	case eJointTypeFixed:
		theta = (is_root) ? state(param_offset + 2) : 0;
		break;
	default:
		assert(false); // unsupported joint
		break;
	}
	return theta;
}

void cKinTree::SetJointTheta(const Eigen::MatrixXd& joint_mat, int joint_id, double theta, Eigen::VectorXd& out_state)
{
	eJointType j_type = GetJointType(joint_mat, joint_id);
	int param_offset = GetParamOffset(joint_mat, joint_id);
	switch (j_type)
	{
	case eJointTypeRevolute:
		out_state(param_offset) = theta;
		break;
	case eJointTypePrismatic:
		out_state(param_offset) = theta;
		break;
	case eJointTypePlanar:
		out_state(param_offset + 2) = theta;
		break;
	default:
		assert(false); // no offset for joint type
		break;
	}
}

double cKinTree::GetJointLimLow(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	return joint_mat(joint_id, cKinTree::eJointDescLimLow);
}

double cKinTree::GetJointLimHigh(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	return joint_mat(joint_id, cKinTree::eJointDescLimHigh);
}

double cKinTree::CalcLinkLength(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	tVector attach_pt = GetScaledAttachPt(joint_mat, joint_id);
	bool is_root = IsRoot(joint_mat, joint_id);
	double len = (is_root) ? 0 : attach_pt.norm();
	return len;
}

tVector cKinTree::GetScaledAttachPt(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	tVector attach_pt = tVector(joint_mat(joint_id, cKinTree::eJointDescAttachX),
		joint_mat(joint_id, cKinTree::eJointDescAttachY),
		joint_mat(joint_id, cKinTree::eJointDescAttachZ), 0);
	double link_scale = GetLinkScale(joint_mat, joint_id);
	return attach_pt * link_scale;
}

double cKinTree::GetLinkScale(const Eigen::MatrixXd& joint_mat, int joint_id)
{
	double scale = joint_mat(joint_id, eJointDescScale);
	return scale;
}

void cKinTree::CalcMaxSubChainLengths(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_lengths)
{
	int num_joints = static_cast<int>(joint_mat.rows());
	out_lengths = Eigen::VectorXd::Zero(num_joints);

	for (int j = num_joints - 1; j >= 0; --j)
	{
		int parent_id = GetParent(joint_mat, j);
		if (parent_id != gInvalidJointID)
		{
			double curr_val = out_lengths(j);
			double len = CalcLinkLength(joint_mat, j);
			double& parent_val = out_lengths(parent_id);

			if (parent_val < len + curr_val)
			{
				parent_val = len + curr_val;
			}
		}
	}
}

void cKinTree::CalcSubTreeMasses(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& body_defs, Eigen::VectorXd& out_masses)
{
	int num_joints = static_cast<int>(joint_mat.rows());
	out_masses = Eigen::VectorXd::Zero(num_joints);

	for (int j = num_joints - 1; j >= 0; --j)
	{
		double& curr_val = out_masses(j);
		double mass = GetBodyMass(body_defs, j);
		curr_val += mass;

		int parent_id = GetParent(joint_mat, j);
		if (parent_id != gInvalidJointID)
		{
			double& parent_val = out_masses(parent_id);
			parent_val += curr_val;
		}
	}
}

bool cKinTree::ParseJoint(const Json::Value& root, tJointDesc& out_joint_desc)
{
	out_joint_desc(eJointDescParent) = -1;
	out_joint_desc(eJointDescScale) = 1;
	out_joint_desc(eJointDescLimLow) = 1;
	out_joint_desc(eJointDescLimHigh) = 0;
	
	for (int i = 0; i < eJointDescMax; ++i)
	{
		const std::string& key = gJointDescKeys[i];
		if (!root[key].isNull())
		{
			out_joint_desc[i] = root[key].asDouble();
		}
	}
	return true;
}

void cKinTree::PostProcessJointMat(Eigen::MatrixXd& out_joint_mat)
{
	int num_joints = GetNumJoints(out_joint_mat);
	int offset = 0;
	for (int j = 0; j < num_joints; ++j)
	{
		int curr_size = GetParamSize(out_joint_mat, j);
		out_joint_mat(j, eJointDescParamOffset) = offset;
		offset += curr_size;
	}
	int root_id = GetRoot(out_joint_mat);

	out_joint_mat(root_id, eJointDescAttachX) = 0;
	out_joint_mat(root_id, eJointDescAttachY) = 0;
	out_joint_mat(root_id, eJointDescAttachZ) = 0;
}

tMatrix cKinTree::ChildParentTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	tMatrix mat;
	eJointType j_type = cKinTree::GetJointType(joint_mat, joint_id);
	switch (j_type)
	{
	case eJointTypeRevolute:
		mat = ChildParentTransRevolute(joint_mat, state, joint_id);
		break;
	case eJointTypePrismatic:
		mat = ChildParentTransPrismatic(joint_mat, state, joint_id);
		break;
	case eJointTypePlanar:
		mat = ChildParentTransPlanar(joint_mat, state, joint_id);
		break;
	case eJointTypeFixed:
		mat = ChildParentTransFixed(joint_mat, state, joint_id);
		break;
	default:
		break;
	}
	
	return mat;
}

tMatrix cKinTree::ParentChildTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	tMatrix mat = ChildParentTrans(joint_mat, state, joint_id);
	mat = cMathUtil::InvRigidMat(mat);
	return mat;
}

tMatrix cKinTree::JointWorldTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	tMatrix m = tMatrix::Identity();
	int curr_id = joint_id;
	while (curr_id != gInvalidJointID)
	{
		tMatrix child_parent_mat = ChildParentTrans(joint_mat, state, curr_id);
		m = child_parent_mat * m;
		curr_id = GetParent(joint_mat, curr_id);
	}

	return m;
}

tMatrix cKinTree::WorldJointTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	tMatrix m = JointWorldTrans(joint_mat, state, joint_id);
	m = cMathUtil::InvRigidMat(m);
	return m;
}

tMatrix cKinTree::BodyWorldTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::MatrixXd& body_defs, int part_id)
{
	tMatrix body_trans = BodyJointTrans(body_defs, part_id);
	tMatrix joint_trans = JointWorldTrans(joint_mat, state, part_id);
	body_trans = joint_trans * body_trans;
	return body_trans;
}

tMatrix cKinTree::BodyJointTrans(const Eigen::MatrixXd& body_defs, int part_id)
{
	tVector attach_pt = GetBodyAttachPt(body_defs, part_id);
	double theta;
	tVector axis;
	GetBodyRotation(body_defs, part_id, axis, theta);
	tVector com = GetBodyLocalCoM(body_defs, part_id);

	tMatrix rot = cMathUtil::RotateMat(axis, theta);
	tMatrix trans = cMathUtil::TranslateMat(attach_pt + com);
	tMatrix m = trans * rot;
	return m;
}

cKinTree::tJointDesc cKinTree::BuildJointDesc(eJointType joint_type, int parent_id, const tVector& attach_pt)
{
	tJointDesc desc;
	desc(eJointDescType) = static_cast<double>(joint_type);
	desc(eJointDescParent) = parent_id;
	desc(eJointDescAttachX) = attach_pt[0];
	desc(eJointDescAttachY) = attach_pt[1];
	desc(eJointDescAttachZ) = attach_pt[2];
	desc(eJointDescScale) = 1;
	desc(eJointDescLinkStiffness) = 1;
	desc(eJointDescLimLow) = 1;
	desc(eJointDescLimHigh) = 0;
	desc(eJointDescParamOffset) = 0;

	return desc;
}

tMatrix cKinTree::ChildParentTransRevolute(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	tVector attach_pt = GetScaledAttachPt(joint_mat, joint_id);
	double theta = GetJointTheta(joint_mat, state, joint_id);
	
	tMatrix R = cMathUtil::RotateMat(tVector(0, 0, 1, 0), theta);
	tMatrix T = cMathUtil::TranslateMat(attach_pt);
	tMatrix mat = T * R;
	return mat;
}

tMatrix cKinTree::ChildParentTransPlanar(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	bool is_root = cKinTree::IsRoot(joint_mat, joint_id);
	tVector attach_pt = GetScaledAttachPt(joint_mat, joint_id);
	double theta = GetJointTheta(joint_mat, state, joint_id);
	tVector offset = GetJointOffset(joint_mat, state, joint_id);

	tMatrix R = cMathUtil::RotateMat(tVector(0, 0, 1, 0), theta);
	tMatrix T0 = cMathUtil::TranslateMat(attach_pt);
	tMatrix T1 = cMathUtil::TranslateMat(offset);

	tMatrix mat; 
	if (is_root)
	{
		mat = T0 * T1 * R;
	}
	else
	{
		mat = T0 * R * T1;
	}
	return mat;
}

tMatrix cKinTree::ChildParentTransFixed(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	bool is_root = cKinTree::IsRoot(joint_mat, joint_id);
	tVector attach_pt = GetScaledAttachPt(joint_mat, joint_id);
	double theta = GetJointTheta(joint_mat, state, joint_id);
	tVector offset = GetJointOffset(joint_mat, state, joint_id);

	tMatrix R = cMathUtil::RotateMat(tVector(0, 0, 1, 0), theta);
	tMatrix T0 = cMathUtil::TranslateMat(attach_pt);
	tMatrix T1 = cMathUtil::TranslateMat(offset);

	tMatrix mat;
	if (is_root)
	{
		mat = T0 * T1 * R;
	}
	else
	{
		mat = T0;
	}
	return mat;
}

tMatrix cKinTree::ChildParentTransPrismatic(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id)
{
	tVector attach_pt = GetScaledAttachPt(joint_mat, joint_id);
	double theta = GetJointTheta(joint_mat, state, joint_id);
	tVector offset = tVector(0, theta, 0, 0);

	tMatrix R = cMathUtil::RotateMat(tVector(0, 0, 1, 0), 0);
	tMatrix T0 = cMathUtil::TranslateMat(attach_pt);
	tMatrix T1 = cMathUtil::TranslateMat(offset);

	tMatrix mat = T0 * R * T1;
	return mat;
}
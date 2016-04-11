#pragma once

#include <vector>
#include <fstream>
#include <json/json.h>

#include "util/MathUtil.h"
#define DISABLE_LINK_SCALE

class cKinTree
{
public:
	// description of the joint tree representing an articulated figure
	enum eJointType
	{
		eJointTypeRevolute,
		eJointTypePlanar,
		eJointTypePrismatic,
		eJointTypeFixed,
		eJointTypeTypeMax
	};

	enum eJointDesc
	{
		eJointDescType,
		eJointDescParent,
		eJointDescAttachX,
		eJointDescAttachY,
		eJointDescAttachZ,
		eJointDescScale,
		eJointDescLinkStiffness,
		eJointDescLimLow,
		eJointDescLimHigh,
		eJointDescParamOffset,
		eJointDescMax
	};
	typedef Eigen::Matrix<double, 1, eJointDescMax> tJointDesc;

	enum eBodyShape
	{
		eBodyShapeBox,
		eBodyShapeCapsule,
		eBodyShapeMax,
		eBodyShapeNULL
	};

	enum eBodyParam
	{
		eBodyParamShape,
		eBodyParamMass,
		eBodyParamAttachX,
		eBodyParamAttachY,
		eBodyParamAttachZ,
		eBodyParamTheta,
		eBodyParam0,
		eBodyParam1,
		eBodyParam2,
		eBodyColorR,
		eBodyColorG,
		eBodyColorB,
		eBodyColorA,
		eBodyParamMax
	};
	typedef Eigen::Matrix<double, 1, eBodyParamMax> tBodyDef;

	enum eDrawShape
	{
		eDrawShapeShape,
		eDrawShapeParentJoint,
		eDrawShapeAttachX,
		eDrawShapeAttachY,
		eDrawShapeAttachZ,
		eDrawShapeTheta,
		eDrawShapeParam0,
		eDrawShapeParam1,
		eDrawShapeParam2,
		eDrawShapeColorR,
		eDrawShapeColorG,
		eDrawShapeColorB,
		eDrawShapeColorA,
		eDrawShapeParamMax
	};
	typedef Eigen::Matrix<double, 1, eDrawShapeParamMax> tDrawShapeDef;

	static const int gInvalidJointID;
	static const int gPosDims;

	static bool HasValidRoot(const Eigen::MatrixXd& joint_mat);
	static tVector GetRootPos(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state);
	static void SetRootPos(const Eigen::MatrixXd& joint_desc, const tVector& pos, Eigen::VectorXd& out_state);
	static void SetRootPosX(const Eigen::MatrixXd& joint_desc, double x, Eigen::VectorXd& out_state);
	static double GetRootTheta(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state);
	static void SetRootTheta(const Eigen::MatrixXd& joint_mat, double theta, Eigen::VectorXd& out_state);

	static tVector CalcJointWorldPos(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static tVector LocalToWorldPos(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int parent_id, const tVector& attach_pt);
	static tVector CalcJointWorldVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::VectorXd& vel, int joint_id);
	static tVector CalcWorldVel(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::VectorXd& vel, int parent_id, const tVector& attach_pt);
	static void CalcJointWorldTheta(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id, tVector& out_axis, double& out_theta);
	static double LocalToWorldTheta(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int parent_id, double local_theta);

	static int GetNumDof(const Eigen::MatrixXd& joint_mat);
	static void ApplyStep(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& step, Eigen::VectorXd& out_pose);

	static Eigen::VectorXi FindJointChain(const Eigen::MatrixXd& joint_mat, int joint_beg, int joint_end);
	static bool IsAncestor(const Eigen::MatrixXd& joint_mat, int child_joint, int ancestor_joint, int& out_len);
	static double CalcChainLength(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXi& chain);

	static void CalcAABB(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, tVector& out_min, tVector& out_max);

	static int GetParamOffset(const Eigen::MatrixXd& joint_mat, int joint_id);
	static int GetParamSize(const Eigen::MatrixXd& joint_mat, int joint_id);
	static Eigen::VectorXd GetJointParams(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int j);
	static void SetJointParams(const Eigen::MatrixXd& joint_mat, int j, const Eigen::VectorXd& params, Eigen::VectorXd& out_state);
	static eJointType GetJointType(const Eigen::MatrixXd& joint_mat, int joint_id);
	static int GetParent(const Eigen::MatrixXd& joint_mat, int joint_id);
	static bool HasParent(const Eigen::MatrixXd& joint_mat, int joint_id);
	static bool IsRoot(const Eigen::MatrixXd& joint_mat, int joint_id);
	static bool IsJointActuated(const Eigen::MatrixXd& joint_mat, int joint_id);

	static tVector GetJointOffset(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static void SetJointOffset(const Eigen::MatrixXd& joint_mat, int joint_id, const tVector& offset, Eigen::VectorXd& out_state);
	static double GetJointTheta(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static void SetJointTheta(const Eigen::MatrixXd& joint_mat, int joint_id, double theta, Eigen::VectorXd& out_state);
	static double GetJointLimLow(const Eigen::MatrixXd& joint_mat, int joint_id);
	static double GetJointLimHigh(const Eigen::MatrixXd& joint_mat, int joint_id);

	static double CalcLinkLength(const Eigen::MatrixXd& joint_mat, int joint_id);
	static tVector GetScaledAttachPt(const Eigen::MatrixXd& joint_mat, int joint_id);
	static double GetLinkScale(const Eigen::MatrixXd& joint_mat, int joint_id);

	// calculates the longest chain in the subtree of each joint
	static void CalcMaxSubChainLengths(const Eigen::MatrixXd& joint_mat, Eigen::VectorXd& out_lengths);
	static void CalcSubTreeMasses(const Eigen::MatrixXd& joint_mat, const Eigen::MatrixXd& body_defs, Eigen::VectorXd& out_masses);
	
	static tMatrix ChildParentTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static tMatrix ParentChildTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static tMatrix JointWorldTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static tMatrix WorldJointTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	
	static bool Load(const Json::Value& root, Eigen::MatrixXd& out_joint_mat);
	static int GetNumJoints(const Eigen::MatrixXd& joint_mat);
	static int GetRoot(const Eigen::MatrixXd& joint_mat);
	static void FindChildren(const Eigen::MatrixXd& joint_mat, int joint_id, Eigen::VectorXi& out_children);

	static bool LoadBodyDefs(const std::string& char_file, Eigen::MatrixXd& out_body_defs);
	static bool ParseBodyDef(const Json::Value& root, tBodyDef& out_def);
	static bool ParseBodyShape(const std::string& str, eBodyShape& out_shape);

	static bool LoadDrawShapeDefs(const std::string& char_file, Eigen::MatrixXd& out_draw_defs);
	static bool ParseDrawShapeDef(const Json::Value& root, tBodyDef& out_def);
	
	static eBodyShape GetBodyShape(const Eigen::MatrixXd& body_defs, int part_id);
	static tVector GetBodyAttachPt(const Eigen::MatrixXd& body_defs, int part_id);
	static void GetBodyRotation(const Eigen::MatrixXd& body_defs, int part_id, tVector& out_axis, double& out_theta);
	static double GetBodyMass(const Eigen::MatrixXd& body_defs, int part_id);
	static tVector GetBodySize(const Eigen::MatrixXd& body_defs, int part_id);
	static tVector GetBodyColor(const Eigen::MatrixXd& body_defs, int part_id);
	static double CalcTotalMass(const Eigen::MatrixXd& body_defs);
	static bool IsValidBody(const Eigen::MatrixXd& body_defs, int part_id);
	static tVector GetBodyLocalCoM(const Eigen::MatrixXd& body_defs, int part_id);

	static int GetDrawShapeParentJoint(const tDrawShapeDef& shape);
	static tVector GetDrawShapeAttachPt(const tDrawShapeDef& shape);
	static void GetDrawShapeRotation(const tDrawShapeDef& shape, tVector& out_axis, double& out_theta);
	static tVector GetDrawShapeColor(const tDrawShapeDef& shape);

	static tVector CalcBodyPartPos(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::MatrixXd& body_defs, int part_id);
	static void CalcBodyPartRotation(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::MatrixXd& body_defs, int part_id, tVector& out_axis, double& out_theta);
	static tMatrix BodyWorldTrans(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, const Eigen::MatrixXd& body_defs, int part_id);
	static tMatrix BodyJointTrans(const Eigen::MatrixXd& body_defs, int part_id);

	static tJointDesc BuildJointDesc(eJointType joint_type, int parent_id, const tVector& attach_pt);

protected:
	static bool ParseJoint(const Json::Value& root, tJointDesc& out_joint_desc);
	static void PostProcessJointMat(Eigen::MatrixXd& out_joint_mat);

	static tMatrix ChildParentTransRevolute(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static tMatrix ChildParentTransPlanar(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static tMatrix ChildParentTransFixed(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);
	static tMatrix ChildParentTransPrismatic(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& state, int joint_id);

};

#include "SimCharacter.h"
#include <iostream>

#include "SimBox.h"
#include "SimCapsule.h"

cSimCharacter::tParams::tParams()
{
	mCharFile = "";
	mStateFile = "";
	mPos = tVector(0, 0, 0, 0);
}

cSimCharacter::cSimCharacter()
	: mWorld(nullptr)
{
	mFriction = 0.9;
	mRootBodyTrans = tMatrix::Identity();
}

cSimCharacter::~cSimCharacter()
{
}

bool cSimCharacter::Init(std::shared_ptr<cWorld> world, const tParams& params)
{
	bool succ = true;
	bool succ_skeleton = cCharacter::Init(params.mCharFile);
	succ &= succ_skeleton;

	mWorld = world;

	bool succ_body = true;
	if (succ_skeleton)
	{
		const tVector& root_pos = params.mPos;
		succ_body = BuildSimBody(params, root_pos);
		LoadDrawShapeDefs(params.mCharFile, mDrawShapeDefs);
	}

	if (params.mStateFile != "")
	{
		bool succ_state = ReadState(params.mStateFile);

		if (!succ_state)
		{
			printf("Failed to load character state from %s\n", params.mStateFile.c_str());
		}
	}

	if (succ)
	{
		RecordDefaultState();
	}

	return succ;
}

void cSimCharacter::Clear()
{
	cCharacter::Clear();
	mJoints.clear();
	mBodyParts.clear();
	mBodyDefs.resize(0, 0);
	mDrawShapeDefs.resize(0, 0);
	mWorld.reset();

	if (HasController())
	{
		mController->Clear();
	}
}


void cSimCharacter::Reset()
{
	cCharacter::Reset();
	if (HasController())
	{
		mController->Reset();
	}
	
	ClearJointTorques();

#if defined(ENABLE_TRAINING)
	ClearEffortBuffer();
#endif
}


void cSimCharacter::Update(double time_step)
{
	BuildPose(mPose);
	ClearJointTorques();

	if (HasController())
	{
		mController->Update(time_step);
	}

	// dont clear torques until next frame since they can be useful for visualization
	UpdateJoints();

#if defined(ENABLE_TRAINING)
	UpdateEffortBuffer(time_step);
#endif
}

tVector cSimCharacter::GetRootPos() const
{
	int root_id = GetRootID();
	const std::shared_ptr<cSimObj>& root = mBodyParts[root_id];
	tVector attach_pt = cKinTree::GetBodyAttachPt(mBodyDefs, root_id);
	tVector pos = root->LocalToWorldPos(-attach_pt);
	return pos;
}

void cSimCharacter::GetRootRotation(tVector& out_axis, double& out_theta) const
{
	int root_id = GetRootID();
	const std::shared_ptr<cSimObj>& root = mBodyParts[root_id];
	
	tMatrix3 root_rot_mat = root->GetLocalToWorldRotMat();

	tMatrix rot_mat = tMatrix::Identity();
	rot_mat.block(0, 0, 3, 3) = mRootBodyTrans.block(0, 0, 3, 3) * root_rot_mat;
	cMathUtil::RotMatToAxisAngle(rot_mat, out_axis, out_theta);
}

tVector cSimCharacter::GetRootVel() const
{
	int root_id = GetRootID();
	const std::shared_ptr<cSimObj>& root = mBodyParts[root_id];
	tVector attach_pt = cKinTree::GetBodyAttachPt(mBodyDefs, root_id);
	tVector vel = root->GetLinearVelocity(-attach_pt);
	return vel;
}

tVector cSimCharacter::GetRootAngVel() const
{
	const std::shared_ptr<cSimObj>& root = mBodyParts[GetRootID()];
	return root->GetAngularVelocity();
}

const Eigen::MatrixXd& cSimCharacter::GetBodyDefs() const
{
	return mBodyDefs;
}

const Eigen::MatrixXd& cSimCharacter::GetDrawShapeDefs() const
{
	return mDrawShapeDefs;
}

void cSimCharacter::SetRootPos(const tVector& pos)
{
	cKinTree::SetRootPos(mJointMat, pos, mPose);
	SetPose(mPose);
}

int cSimCharacter::GetNumBodyParts() const
{
	return static_cast<int>(mBodyParts.size());
}

void cSimCharacter::BuildPose(Eigen::VectorXd& out_pose) const
{
	const double link_scale = 1;
	int num_joints = GetNumJoints();
	int num_dof = cKinTree::GetNumDof(mJointMat);
	out_pose = Eigen::VectorXd(num_dof);

	tVector root_pos = GetRootPos();
	out_pose.block(0, 0, cKinTree::gPosDims, 1) = root_pos.block(0, 0, cKinTree::gPosDims, 1);

	for (int j = 0; j < num_joints; ++j)
	{
		int idx = cKinTree::gPosDims + j;
		double theta = 0;
		tVector axis;
		if (j == GetRootID())
		{
			// note for now theta all with respect to positive z axis
			GetRootRotation(axis, theta);
			theta = (axis[2] >= 0) ? theta : -theta;
		}
		else if (mJoints[j].IsValid())
		{
			CalcJointRotation(j, axis, theta);
		}
		out_pose(idx) = theta;
	}
}

void cSimCharacter::BuildVel(Eigen::VectorXd& out_vel) const
{
	const double link_scale_vel = 0;
	int num_joints = GetNumJoints();
	int num_dof = cKinTree::GetNumDof(mJointMat);
	out_vel = Eigen::VectorXd(num_dof);

	tVector root_vel = GetRootVel();
	out_vel.block(0, 0, cKinTree::gPosDims, 1) = root_vel.block(0, 0, cKinTree::gPosDims, 1);

	for (int j = 0; j < num_joints; ++j)
	{
		int idx = cKinTree::gPosDims + j;
		double d_theta = 0;
		if (j == GetRootID())
		{
			// note only consider z
			const tVector rot_axis = tVector(0, 0, 1, 0);
			tVector ang_vel = GetRootAngVel();
			d_theta = rot_axis.dot(ang_vel);
		}
		else if (mJoints[j].IsValid())
		{
			const cJoint& joint = mJoints[j];
			tVector joint_vel = joint.CalcJointVelRel();
			const tVector& axis = joint.GetAxisRel();
			d_theta = axis.dot(joint_vel);
		}
		out_vel(idx) = d_theta;
	}
}

void cSimCharacter::SetVel(const Eigen::VectorXd& vel)
{
	assert(vel.size() == GetNumDof());

	// uses spatial algebra to convert generalized coordinates into
	// velocities of each body part
	int num_joints = GetNumJoints();
	Eigen::VectorXi visited = Eigen::VectorXi::Zero(num_joints);

	// use 4d vectors just for convenience
	const int dim = 3;
	Eigen::MatrixXd world_omegas = Eigen::MatrixXd(num_joints, dim);
	Eigen::MatrixXd world_vels = Eigen::MatrixXd(num_joints, dim);

	const int joint_vel_offset = cKinTree::gPosDims;
	int root_id = GetRootID();

	for (int j = 0; j < num_joints; ++j)
	{
		if (IsValidBodyPart(j))
		{
			Eigen::Vector3d world_omega = Eigen::Vector3d::Zero();
			Eigen::Vector3d world_vel = Eigen::Vector3d::Zero();

			int curr_id = j;
			while (true)
			{
				bool has_visited = (visited[curr_id] != 0);
				if (has_visited)
				{
					const Eigen::Vector3d& curr_omega = world_omegas.row(curr_id);
					const Eigen::Vector3d& curr_vel = world_vels.row(curr_id);
					world_omega += curr_omega;
					world_vel += curr_vel;
					break;
				}
				else
				{
					int idx = joint_vel_offset + curr_id;
					double joint_vel = vel(idx);
					auto& curr_part = GetBodyPart(curr_id);

					tMatrix3 E = curr_part->GetLocalToWorldRotMat();
					tMatrix3 E_inv = E.transpose();

					Eigen::Vector3d local_omega = Eigen::Vector3d(0, 0, joint_vel);
					Eigen::Vector3d local_vel = Eigen::Vector3d::Zero();
					if (curr_id == root_id)
					{
						local_vel.block(0, 0, cKinTree::gPosDims, 1) = vel.block(0, 0, cKinTree::gPosDims, 1);
					}
					// convert vel to local coordinates
					local_vel = E_inv * local_vel;

					// convert spatial vector to world coordinates
					tVector joint_pos = CalcJointPos(curr_id);
					Eigen::Vector3d r = E_inv * -Eigen::Vector3d(joint_pos[0], joint_pos[1], joint_pos[2]);
					
					Eigen::Vector3d curr_omega = E * local_omega;
					Eigen::Vector3d curr_vel = E * (-r.cross(local_omega) + local_vel);

					world_omega += curr_omega;
					world_vel += curr_vel;
				}

				curr_id = cKinTree::GetParent(mJointMat, curr_id);
				if (curr_id == cKinTree::gInvalidJointID)
				{
					break;
				}
			}

			auto& part = GetBodyPart(j);

			visited(j) = 1;
			world_omegas.row(j) = world_omega;
			world_vels.row(j) = world_vel;

			// calculate the spatial velocity at the center of mass
			tVector com_pos = part->GetPos();
			Eigen::Vector3d r = Eigen::Vector3d(com_pos[0], com_pos[1], com_pos[2]);
			Eigen::Vector3d com_omega = world_omega;
			Eigen::Vector3d com_vel = -r.cross(world_omega) + world_vel;

			part->SetAngularVelocity(tVector(com_omega[0], com_omega[1], com_omega[2], 0));
			part->SetLinearVelocity(tVector(com_vel[0], com_vel[1], com_vel[2], 0));
		}
	}
}

tVector cSimCharacter::CalcJointPos(int joint_id) const
{
	const cJoint& joint = mJoints[joint_id];
	tVector pos;
	
	if (joint.IsValid())
	{
		pos = joint.GetPos();
	}
	else if (joint_id == GetRootID())
	{
		pos = GetRootPos();
	}
	else
	{
		int parent_id = cKinTree::GetParent(mJointMat, joint_id);
		assert(parent_id != cKinTree::gInvalidJointID);
		assert(IsValidBodyPart(parent_id));

		tVector attach_pt = cKinTree::GetScaledAttachPt(mJointMat, joint_id);
		tVector part_attach_pt = cKinTree::GetBodyAttachPt(mBodyDefs, parent_id);
		attach_pt -= part_attach_pt;

		const auto& parent_part = GetBodyPart(parent_id);
		pos = parent_part->LocalToWorldPos(attach_pt);
	}
	return pos;
}

tVector cSimCharacter::CalcJointVel(int joint_id) const
{
	const cJoint& joint = mJoints[joint_id];
	tVector vel;

	if (joint.IsValid())
	{
		vel = joint.GetVel();
	}
	else if (joint_id == GetRootID())
	{
		vel = GetRootVel();
	}
	else
	{
		int parent_id = cKinTree::GetParent(mJointMat, joint_id);
		assert(parent_id != cKinTree::gInvalidJointID);
		assert(IsValidBodyPart(parent_id));

		tVector attach_pt = cKinTree::GetScaledAttachPt(mJointMat, joint_id);
		tVector part_attach_pt = cKinTree::GetBodyAttachPt(mBodyDefs, parent_id);
		attach_pt -= part_attach_pt;

		const auto& parent_part = GetBodyPart(parent_id);
		vel = parent_part->GetLinearVelocity(attach_pt);
	}

	return vel;
}

void cSimCharacter::CalcJointWorldRotation(int joint_id, tVector& out_axis, double& out_theta) const
{
	if (IsValidBodyPart(joint_id))
	{
		mBodyParts[joint_id]->GetRotation(out_axis, out_theta);
	}
	else
	{
		int parent_id = cKinTree::GetParent(mJointMat, joint_id);
		assert(parent_id != cKinTree::gInvalidJointID);
		assert(IsValidBodyPart(parent_id));

		out_axis = tVector(0, 0, 1, 0);
		const auto& parent_part = GetBodyPart(parent_id);
		auto rot = parent_part->GetLocalToWorldRotMat();
		out_axis.block(0, 0, 3, 1) = rot * out_axis.block(0, 0, 3, 1);
		out_theta = 0;
	}
}

tVector cSimCharacter::CalcCOM() const
{
	tVector com = tVector::Zero();
	double total_mass = 0;
	for (int i = 0; i < static_cast<int>(mBodyParts.size()); ++i)
	{
		if (IsValidBodyPart(i))
		{
			const auto& part = mBodyParts[i];
			double mass = part->GetMass();
			tVector curr_com = part->GetPos();

			com += mass * curr_com;
			total_mass += mass;
		}
	}
	com /= total_mass;
	return com;
}

tVector cSimCharacter::CalcCOMVel() const
{
	tVector com_vel = tVector::Zero();
	double total_mass = 0;
	for (int i = 0; i < static_cast<int>(mBodyParts.size()); ++i)
	{
		if (IsValidBodyPart(i))
		{
			const auto& part = mBodyParts[i];
			double mass = part->GetMass();
			tVector curr_vel = part->GetLinearVelocity();

			com_vel += mass * curr_vel;
			total_mass += mass;
		}
	}
	com_vel /= total_mass;
	return com_vel;
}

void cSimCharacter::CalcAABB(tVector& out_min, tVector& out_max) const
{
	out_min[0] = std::numeric_limits<double>::infinity();
	out_min[1] = std::numeric_limits<double>::infinity();
	out_min[2] = std::numeric_limits<double>::infinity();

	out_max[0] = -std::numeric_limits<double>::infinity();
	out_max[1] = -std::numeric_limits<double>::infinity();
	out_max[2] = -std::numeric_limits<double>::infinity();

	for (int i = 0; i < GetNumBodyParts(); ++i)
	{
		if (IsValidBodyPart(i))
		{
			const auto& part = GetBodyPart(i);

			tVector curr_min = tVector::Zero();
			tVector curr_max = tVector::Zero();
			part->CalcAABB(curr_min, curr_max);

			out_min = out_min.cwiseMin(curr_min);
			out_max = out_max.cwiseMax(curr_max);
		}
	}
}

const cJoint& cSimCharacter::GetJoint(int joint_id) const
{
	return mJoints[joint_id];
}

cJoint& cSimCharacter::GetJoint(int joint_id)
{
	return mJoints[joint_id];
}

const std::shared_ptr<cSimObj>& cSimCharacter::GetBodyPart(int idx) const
{
	return mBodyParts[idx];
}

std::shared_ptr<cSimObj>& cSimCharacter::GetBodyPart(int idx)
{
	return mBodyParts[idx];
}

const std::shared_ptr<cSimObj>& cSimCharacter::GetRootPart() const
{
	int root_idx = GetRootID();
	return mBodyParts[root_idx];
}

std::shared_ptr<cSimObj> cSimCharacter::GetRootPart()
{
	int root_idx = GetRootID();
	return mBodyParts[root_idx];
}

void cSimCharacter::RegisterContacts(int contact_flag, int filter_flag)
{
	for (int i = 0; i < static_cast<int>(mBodyParts.size()); ++i)
	{
		if (IsValidBodyPart(i))
		{
			std::shared_ptr<cSimObj>& part = mBodyParts[i];
			part->RegisterContact(contact_flag, filter_flag);
		}
	}
}

bool cSimCharacter::HasFallen() const
{
	return false;
}

bool cSimCharacter::HasStumbled() const
{
	return false;
}

bool cSimCharacter::HasExploded() const
{
	const double dist_threshold = 0.02 * 0.02;
	bool exploded = false;

	int num_joints = GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		const cJoint& joint = GetJoint(j);
		if (joint.IsValid())
		{
			const auto& parent = joint.GetParent();
			const auto& child = joint.GetChild();
			const tVector& anchor_parent = joint.GetParentAnchor();
			const tVector& anchor_child = joint.GetChildAnchor();

			tVector parent_pos = parent->LocalToWorldPos(anchor_parent);
			tVector child_pos = child->LocalToWorldPos(anchor_child);

			tVector delta = child_pos - parent_pos;
			double dist = delta.squaredNorm();

			if (dist > dist_threshold)
			{
				exploded = true;
				break;
			}
		}
	}

	return exploded;
}

bool cSimCharacter::IsEndEffector(int idx) const
{
	return false;
}

bool cSimCharacter::IsInContact() const
{
	for (int i = 0; i < GetNumBodyParts(); ++i)
	{
		if (IsValidBodyPart(i))
		{
			if (IsInContact(i))
			{
				return true;
			}
		}
	}
	return false;
}

bool cSimCharacter::IsInContact(int idx) const
{
	return GetBodyPart(idx)->IsInContact();
}

tVector cSimCharacter::GetContactPt(int idx) const
{
	return GetBodyPart(idx)->GetContactPt();
}

int cSimCharacter::GetState() const
{
	if (HasController())
	{
		return mController->GetState();
	}
	assert(false);
	return 0;
}

double cSimCharacter::GetPhase() const
{
	if (HasController())
	{
		return mController->GetPhase();
	}
	assert(false);
	return 0;
}


void cSimCharacter::SetController(std::shared_ptr<cCharController> ctrl)
{
	RemoveController();
	mController = ctrl;
}

void cSimCharacter::RemoveController()
{
	if (HasController())
	{
		mController.reset();
	}
}

bool cSimCharacter::HasController() const
{
	return mController != nullptr;
}

const std::shared_ptr<cCharController>& cSimCharacter::GetController()
{
	return mController;
}

const std::shared_ptr<cCharController>& cSimCharacter::GetController() const
{
	return mController;
}

void cSimCharacter::EnableController(bool enable)
{
	if (HasController())
	{
		mController->SetActive(enable);
	}
}

void cSimCharacter::ApplyControlForces(const Eigen::VectorXd& tau)
{
	assert(tau.size() == GetNumDof());
	for (int j = 0; j < GetNumJoints(); ++j)
	{
		cJoint& joint = GetJoint(j);
		if (joint.IsValid())
		{
			int param_offset = GetParamOffset(j);
			int param_size = GetParamSize(j);
			auto curr_tau = tau.segment(param_offset, param_size);
			assert(curr_tau.size() == 1);
			tVector torque = tVector(0, 0, curr_tau[0], 0);

			joint.AddTorque(torque);
		}
	}
}

void cSimCharacter::PlayPossum()
{
	if (HasController())
	{
		mController->SetMode(cController::eModePassive);
	}
}


void cSimCharacter::SetPose(const Eigen::VectorXd& pose)
{
	cCharacter::SetPose(pose);

	for (int i = 0; i < static_cast<int>(mBodyParts.size()); ++i)
	{
		if (IsValidBodyPart(i))
		{
			auto& curr_part = mBodyParts[i];
			tVector axis;
			double theta;
			cKinTree::CalcBodyPartRotation(mJointMat, mPose, mBodyDefs, i, axis, theta);
			tVector attach_pt = cKinTree::CalcBodyPartPos(mJointMat, mPose, mBodyDefs, i);

			curr_part->SetPos(attach_pt);
			curr_part->SetRotation(axis, theta);
		}
	}
}

bool cSimCharacter::BuildSimBody(const tParams& params, const tVector& root_pos)
{
	bool succ = true;
	succ = LoadBodyDefs(params.mCharFile, mBodyDefs);

	int num_joints = GetNumJoints();
	mBodyParts.resize(num_joints);
	for (int j = 0; j < num_joints; ++j)
	{
		auto& curr_part = mBodyParts[j];
		BuildBodyPart(j, root_pos, mFriction, curr_part);

		if (curr_part != nullptr)
		{
			curr_part->UpdateContact(cWorld::eContactFlagCharacter, cWorld::eContactFlagAll);
			curr_part->DisableDeactivation();
		}
	}

	BuildConstraints(params.mPlaneCons);

#if defined(ENABLE_TRAINING)
	mEffortBuffer.resize(GetNumJoints());
	ClearEffortBuffer();
#endif

	int root_id = GetRootID();
	mRootBodyTrans = cMathUtil::InvRigidMat(cKinTree::BodyJointTrans(mBodyDefs, root_id));

	return true;
}

bool cSimCharacter::LoadBodyDefs(const std::string& char_file, Eigen::MatrixXd& out_body_defs) const
{
	bool succ = cKinTree::LoadBodyDefs(char_file, out_body_defs);
	int num_joints = GetNumJoints();
	int num_body_defs = static_cast<int>(out_body_defs.rows());
	assert(num_joints == num_body_defs);
	return succ;
}

bool cSimCharacter::LoadDrawShapeDefs(const std::string& char_file, Eigen::MatrixXd& out_draw_defs) const
{
	bool succ = cKinTree::LoadDrawShapeDefs(char_file, out_draw_defs);
	return succ;
}


void cSimCharacter::BuildBodyPart(int part_id, const tVector& root_pos, double friction, std::shared_ptr<cSimObj>& out_part)
{
	cKinTree::eBodyShape shape = cKinTree::GetBodyShape(mBodyDefs, part_id);
	switch (shape)
	{
		case cKinTree::eBodyShapeBox:
		{
			BuildBoxPart(part_id, root_pos, friction, out_part);
			break;
		}
		case cKinTree::eBodyShapeCapsule:
		{
			BuildCapsulePart(part_id, root_pos, friction, out_part);
			break;
		}
		default:
			out_part = nullptr;
			break;
	}
}

void cSimCharacter::BuildBoxPart(int part_id, const tVector& root_pos, double friction,
								std::shared_ptr<cSimObj>& out_part)
{
	tMatrix com_world_trans = cKinTree::BodyWorldTrans(mJointMat, mPose, mBodyDefs, part_id);
	tVector pos = com_world_trans * tVector(0, 0, 0, 1);
	pos += root_pos;

	tVector axis;
	double theta;
	cMathUtil::RotMatToAxisAngle(com_world_trans, axis, theta);

	cSimBox::tParams params;
	params.mSize = cKinTree::GetBodySize(mBodyDefs, part_id);
	params.mPos = pos;
	params.mAxis = axis;
	params.mTheta = theta;
	params.mFriction = friction;
	params.mMass = cKinTree::GetBodyMass(mBodyDefs, part_id);
	std::unique_ptr<cSimBox> box = std::unique_ptr<cSimBox>(new cSimBox());

	short col_group = GetPartColGroup(part_id);
	short col_mask = GetPartColMask(part_id);
	box->SetColGroup(col_group);
	box->SetColMask(col_mask);

	box->Init(mWorld, params);

	out_part = std::move(box);
}

void cSimCharacter::BuildCapsulePart(int part_id, const tVector& root_pos, double friction,
								std::shared_ptr<cSimObj>& out_part)
{
	tMatrix com_world_trans = cKinTree::BodyWorldTrans(mJointMat, mPose, mBodyDefs, part_id);
	tVector pos = com_world_trans * tVector(0, 0, 0, 1);
	pos += root_pos;

	tVector axis;
	double theta;
	cMathUtil::RotMatToAxisAngle(com_world_trans, axis, theta);

	cSimCapsule::tParams params;
	tVector size_params = cKinTree::GetBodySize(mBodyDefs, part_id);
	params.mHeight = size_params(1);
	params.mRadius = size_params(0);
	params.mPos = pos;
	params.mAxis = axis;
	params.mTheta = theta;
	params.mFriction = friction;
	params.mMass = cKinTree::GetBodyMass(mBodyDefs, part_id);
	std::unique_ptr<cSimCapsule> box = std::unique_ptr<cSimCapsule>(new cSimCapsule());

	short col_group = GetPartColGroup(part_id);
	short col_mask = GetPartColMask(part_id);
	box->SetColGroup(col_group);
	box->SetColMask(col_mask);

	box->Init(mWorld, params);

	out_part = std::move(box);
}

void cSimCharacter::BuildConstraints(cWorld::ePlaneCons plane_cons)
{
	int num_joints = GetNumJoints();
	mJoints.clear();
	mJoints.resize(num_joints);
	tVector root_pos = GetRootPos();

	Eigen::VectorXd default_pose = Eigen::VectorXd::Zero(mPose.size());

	for (int j = 0; j < num_joints; ++j)
	{
		int parent_id = cKinTree::GetParent(mJointMat, j);
		tVector joint_pos = cCharacter::CalcJointPos(j);
		joint_pos += root_pos;
		std::shared_ptr<cSimObj>& curr_part = mBodyParts[j];

		bool valid_part = IsValidBodyPart(j);
		if (parent_id != cKinTree::gInvalidJointID)
		{
			if (valid_part)
			{
				tVector joint_axis = tVector(0, 0, 1, 0);

				tMatrix curr_body_mat = cKinTree::BodyJointTrans(mBodyDefs, j);
				tMatrix child_parent_mat = cKinTree::ParentChildTrans(mJointMat, default_pose, j);
				tMatrix parent_body_mat = cMathUtil::InvRigidMat(cKinTree::BodyJointTrans(mBodyDefs, parent_id));
				tMatrix child_parent_body_mat = cMathUtil::InvRigidMat(parent_body_mat) * child_parent_mat * curr_body_mat;

				tVector ref_axis;
				double ref_theta;
				cMathUtil::RotMatToAxisAngle(child_parent_body_mat, ref_axis, ref_theta);
				ref_theta = -ref_theta;

				const cKinTree::tJointDesc joint_desc = mJointMat.row(j);
				std::shared_ptr<cSimObj>& parent = mBodyParts[parent_id];

				tVector pos_child = curr_part->WorldToLocalPos(joint_pos);
				tVector pos_parent = parent->WorldToLocalPos(joint_pos);

				cWorld::tJointParams cons_params;
				// cons_params.mType = cWorld::eJointTypeHinge;
				cons_params.mType = (cWorld::eJointType) (int) joint_desc(cKinTree::eJointDescType);
				// cons_params.mType = curr_part->GetType(); // wrong kind of type
				cons_params.mAnchor0 = pos_parent;
				cons_params.mAnchor1 = pos_child;
				cons_params.mAxis = joint_axis;
				cons_params.mLimLow = joint_desc(cKinTree::eJointDescLimLow);
				cons_params.mLimHigh = joint_desc(cKinTree::eJointDescLimHigh);
				cons_params.mEnableAdjacentCollision = false;
				cons_params.mRefTheta = ref_theta;
				curr_part->GetRotation(cons_params.mAnchorAxis1, cons_params.mAnchorTheta1);

				const double torque_lim = 300;
				cJoint& curr_joint = mJoints[j];
				curr_joint.Init(mWorld, parent, curr_part, cons_params);
				curr_joint.SetTorqueLimit(torque_lim);
			}
		}
		
		if (valid_part)
		{
			tVector linear_factor;
			tVector angular_factor;
			BuildConsFactor(plane_cons, j, linear_factor, angular_factor);
			curr_part->Constrain(linear_factor, angular_factor);
		}
	}
}

void cSimCharacter::BuildConsFactor(cWorld::ePlaneCons plane_cons, int joint_id, tVector& out_linear_factor, tVector& out_angular_factor) const
{
	cKinTree::eJointType joint_type = cKinTree::GetJointType(mJointMat, joint_id);
	bool is_root = cKinTree::IsRoot(mJointMat, joint_id);

	mWorld->BuildConsFactor(plane_cons, out_linear_factor, out_angular_factor);
	if (is_root)
	{
		tVector root_linear_factor;
		tVector root_angular_factor;
		BuildRootConsFactor(joint_type, root_linear_factor, root_angular_factor);

		out_linear_factor = out_linear_factor.cwiseProduct(root_linear_factor);
		out_angular_factor = out_angular_factor.cwiseProduct(root_angular_factor);
	}
}

void cSimCharacter::BuildRootConsFactor(cKinTree::eJointType joint_type, tVector& out_linear_factor, tVector& out_angular_factor) const
{
	out_linear_factor = tVector::Ones();
	out_angular_factor = tVector::Ones();

	switch (joint_type)
	{
	case cKinTree::eJointTypeRevolute:
		out_linear_factor = tVector::Zero();
		out_angular_factor = tVector(0, 0, 1, 0);
		break;
	case cKinTree::eJointTypePlanar:
		out_linear_factor = tVector(1, 1, 0, 0);
		out_angular_factor = tVector(0, 0, 1, 0);
		break;
	case cKinTree::eJointTypePrismatic:
		out_linear_factor = tVector(1, 1, 0, 0);
		out_angular_factor = tVector(0, 0, 1, 0);
		break;
	case cKinTree::eJointTypeFixed:
		out_linear_factor = tVector::Zero();
		out_angular_factor = tVector::Zero();
		break;
	default:
		assert(false); // unsupported joint type
		break;
	}
}

bool cSimCharacter::IsValidBodyPart(int idx) const
{
	return mBodyParts[idx] != nullptr;
}

void cSimCharacter::CalcJointRotation(int joint_id, tVector& out_axis, double& out_theta) const
{
	const cJoint& joint = mJoints[joint_id];
	if (joint.IsValid())
	{
		joint.CalcRotation(out_axis, out_theta);
	}
	else
	{
		out_axis = tVector(0, 0, 1, 0);
		out_theta = cKinTree::GetJointTheta(mJointMat, mPose, joint_id);
	}
}

tMatrix cSimCharacter::BuildJointWorldTrans(int joint_id) const
{
	const cJoint& joint = mJoints[joint_id];
	if (joint.IsValid())
	{
		return joint.BuildWorldTrans();
	}
	else
	{
		return cCharacter::BuildJointWorldTrans(joint_id);
	}
}

void cSimCharacter::ClearJointTorques()
{
	int num_joints = GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		cJoint& joint = GetJoint(j);
		if (joint.IsValid())
		{
			joint.ClearTorque();
		}
		
	}
}

void cSimCharacter::UpdateJoints()
{
	int num_joints = GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		cJoint& joint = GetJoint(j);
		if (joint.IsValid())
		{
			// if (joint.) would be nice to check joint type and apply force or troque here.
			joint.ApplyTorque();
		}
	}
}

short cSimCharacter::GetPartColGroup(int part_id) const
{
	return cContactManager::gFlagAll;
}

short cSimCharacter::GetPartColMask(int part_id) const
{
	return cContactManager::gFlagAll;
}

tVector cSimCharacter::GetPartColor(int part_id) const
{
	return cKinTree::GetBodyColor(mBodyDefs, part_id);
}

bool cSimCharacter::HasDrawShapes() const
{
	return mDrawShapeDefs.size() > 0;
}

double cSimCharacter::GetPoseJointWeight(int joint_id) const
{
	return 1;
}

const std::shared_ptr<cWorld>& cSimCharacter::GetWorld() const
{
	return mWorld;
}

#if defined(ENABLE_TRAINING)
// effort measured as sum of squared torques
double cSimCharacter::CalcEffort() const
{
	double effort = 0;
	for (int j = 0; j < GetNumJoints(); ++j)
	{
		const cJoint& joint = mJoints[j];
		if (joint.IsValid())
		{
			effort += mEffortBuffer[j];
		}
	}
	return effort;
}

void cSimCharacter::ClearEffortBuffer()
{
	for (int j = 0; j < GetNumJoints(); ++j)
	{
		mEffortBuffer[j] = 0;
	}
}

void cSimCharacter::UpdateEffortBuffer(double time_step)
{
	int num_joints = GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		const cJoint& joint = GetJoint(j);
		if (joint.IsValid())
		{
			const tVector& torque = joint.GetTorque();
			double effort = torque.squaredNorm();
			mEffortBuffer[j] += effort * time_step;
		}
	}
}
#endif // ENABLE_TRAINING

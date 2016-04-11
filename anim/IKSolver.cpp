#include "IKSolver.h"
#include <iostream>
#include "anim/KinTree.h"

const int gPosDims = 2;

cIKSolver::tProblem::tProblem()
{
	mMaxIter = 128;
	mTol = 0.000001;
	mDamp = 0.1;
	mClampDist = 0.1;
	mRestStateGain = 0;
}

cIKSolver::tProblem::~tProblem()
{
}

std::default_random_engine cIKSolver::gRandGen(static_cast<unsigned long int>(time(NULL)));
std::uniform_real_distribution<double> cIKSolver::gRandDoubleDist(0, 1);


void cIKSolver::PrintMatrix(const Eigen::MatrixXd& mat)
{
	printf("_________________\n");
	std::cout << mat << std::endl;
}

void cIKSolver::Solve(const tProblem& prob, tSolution& out_soln)
{
	Eigen::MatrixXd joint_desc = prob.mJointDesc;
	const Eigen::MatrixXd& cons_desc = prob.mConsDesc;

	assert(joint_desc.cols() == cKinTree::eJointDescMax);
	assert(cons_desc.cols() == eConsDescMax);

	int root_id = cKinTree::GetRoot(joint_desc);
	if (root_id == cKinTree::gInvalidJointID)
	{
		printf("Failed to find root in joint tree description.\n");
		return;
	}

	Eigen::VectorXd curr_pose = prob.mPose;
	double prev_obj = 0;

	int i = 0;
	const double discount = 0.5;
	double delta_obj_acc = 0;
	for (i = 0; i < prob.mMaxIter; ++i)
	{
		//StepWeighted(cons_desc, prob, joint_desc, curr_pose, curr_pose);
		StepHybrid(cons_desc, prob, joint_desc, curr_pose);

		// measure change in objective function using the cumulative discounted change
		double curr_obj = CalcObjVal(joint_desc, curr_pose, cons_desc);
		double delta_obj = curr_obj - prev_obj;
		delta_obj = std::abs(delta_obj);
		if (i == 0)
		{
			delta_obj_acc = delta_obj;
		}
		else 
		{
			delta_obj_acc = delta_obj + discount * delta_obj_acc;

			if (delta_obj_acc < std::abs(prob.mTol))
			{
				break;
			}
		}

		prev_obj = curr_obj;
	}

	//printf("Iter: %i\n", i);
	out_soln.mState = curr_pose;
}

double cIKSolver::CalcObjVal(const Eigen::MatrixXd &joint_desc, const Eigen::VectorXd& pose, const Eigen::MatrixXd& cons_desc)
{
	// objective function is the 2-norm of the constraint violations
	double obj_val = 0;
	Eigen::VectorXd err;
	for (int c = 0; c < cons_desc.rows(); ++c)
	{
		const tConsDesc& curr_cons = cons_desc.row(c);
		double weight = curr_cons(eConsDesc::eConsDescWeight);
		err = BuildErr(joint_desc, pose, curr_cons);
		obj_val += weight * err.squaredNorm();
	}
	return obj_val;
}

void cIKSolver::StepWeighted(const Eigen::MatrixXd& cons_desc, const tProblem& prob, Eigen::MatrixXd& joint_desc,
							Eigen::VectorXd& out_pose)
{
	const double priority_decay = 0.5f;
	int num_dof = cKinTree::GetNumDof(joint_desc);
	Eigen::VectorXd err;
	Eigen::MatrixXd J;
	Eigen::MatrixXd J_weighted = Eigen::MatrixXd::Zero(num_dof, num_dof);
	Eigen::VectorXd Jt_err_weighted = Eigen::VectorXd::Zero(num_dof);

	int num_joints = static_cast<int>(joint_desc.rows());
	int num_cons = static_cast<int>(cons_desc.rows());

	double clamp_dist = prob.mClampDist;
	double damp = prob.mDamp;

	for (int c = 0; c < num_cons; ++c)
	{
		const tConsDesc& curr_cons = cons_desc.row(c);
		err = BuildErr(joint_desc, out_pose, curr_cons, clamp_dist);
		J = BuildJacob(joint_desc, out_pose, curr_cons);

		double curr_priority = cons_desc(c, eConsDescPriority);
		double weight = curr_cons(eConsDescWeight);
		weight *= std::pow(priority_decay, curr_priority);

		J_weighted += weight * J.transpose() * J;
		Jt_err_weighted += weight * J.transpose() * err;
	}
	
	// link scaling is damped separately according to stiffness
	for (int i = 0; i < gPosDims + num_joints; ++i)
	{
		J_weighted(i, i) += damp;
	}

#if !defined(DISABLE_LINK_SCALE)
	// damp link scaling according to stiffness
	for (int i = 0; i < num_joints; ++i)
	{
		double d_scale = 1.f - joint_desc(i, cKinTree::eJointDescScale);
		double link_stiffness = joint_desc(i, cKinTree::eJointDescLinkStiffness);

		int idx = gPosDims + num_joints + i;
		Jt_err_weighted(idx) += link_stiffness * d_scale;
		J_weighted(idx, idx) += link_stiffness;
	}
#endif
	
	Eigen::VectorXd x = J_weighted.lu().solve(Jt_err_weighted);
	cKinTree::ApplyStep(joint_desc, x, out_pose);
}


void cIKSolver::StepHybrid(const Eigen::MatrixXd& cons_desc, const tProblem& prob, Eigen::MatrixXd& joint_desc,
							Eigen::VectorXd& out_pose)
{
	const int num_dof = cKinTree::GetNumDof(joint_desc);
	const int num_joints = static_cast<int>(joint_desc.rows());
	const int num_cons = static_cast<int>(cons_desc.rows());

	Eigen::VectorXd err;
	Eigen::MatrixXd J;
	Eigen::MatrixXd J_weighted_buff = Eigen::MatrixXd::Zero(num_dof, num_dof);
	Eigen::VectorXd Jt_err_weighted_buff = Eigen::VectorXd::Zero(num_dof);
	Eigen::MatrixXd N = Eigen::MatrixXd::Identity(num_dof, num_dof);
	Eigen::VectorXi chain_joints(num_joints); // keeps track of joints in Ik chain

	double clamp_dist = prob.mClampDist;
	double damp = prob.mDamp;

	int min_priority = std::numeric_limits<int>::max();
	int max_priority = std::numeric_limits<int>::min();

	for (int c = 0; c < num_cons; ++c)
	{
		int curr_priority = static_cast<int>(cons_desc(c, eConsDescPriority));
		min_priority = std::min(min_priority, curr_priority);
		max_priority = std::max(max_priority, curr_priority);
	}

	for (int p = min_priority; p <= max_priority; ++p)
	{
		int curr_num_dof = static_cast<int>(N.cols());
		auto J_weighted = J_weighted_buff.block(0, 0, curr_num_dof, curr_num_dof);
		auto Jt_err_weighted = Jt_err_weighted_buff.block(0, 0, curr_num_dof, 1);

		J_weighted.setZero();
		Jt_err_weighted.setZero();

		chain_joints.setZero();
		
		int num_valid_cons = 0;
		for (int c = 0; c < num_cons; ++c)
		{
			const tConsDesc& curr_cons = cons_desc.row(c);
			int curr_priority = static_cast<int>(curr_cons(eConsDescPriority));

			if (curr_priority == p)
			{
				++num_valid_cons;
				err = BuildErr(joint_desc, out_pose, curr_cons, clamp_dist);
				J = BuildJacob(joint_desc, out_pose, curr_cons);

#if !defined(DISABLE_LINK_SCALE)
				for (int i = 0; i < num_joints; ++i)
				{
					// use entries in the jacobian to figure out if a joint is on the
					// link chain from the root to the constrained end effectors
					// this ignores the root which should not have any scaling
					int scale_idx = gPosDims + num_joints + i;
					int theta_idx = gPosDims + i;
					double scaling = J.col(scale_idx).squaredNorm();
					if (scaling > 0)
					{
						chain_joints(i) = 1;
					}
				}
#endif
				J = J * N;

				double weight = curr_cons(eConsDescWeight);
				J_weighted += weight * J.transpose() * J;
				Jt_err_weighted += weight * J.transpose() * err;
			}
		}

		if (num_valid_cons > 0)
		{
			// apply damping
			// a little more tricky with the null space

			auto N_block = N.block(0, 0, gPosDims + num_joints, N.cols());
			J_weighted += damp * N.transpose() * N;

#if !defined(DISABLE_LINK_SCALE)
			// damp link scaling according to stiffness
			for (int i = 0; i < num_joints; ++i)
			{
				// only scale links that are part of the IK chain
				if (chain_joints(i) == 1)
				{
					int idx = gPosDims + num_joints + i;
					const Eigen::VectorXd& N_row = N.row(idx);

					double d_scale = 1.f - joint_desc(i, cKinTree::eJointDescScale);
					double link_stiffness = joint_desc(i, cKinTree::eJointDescLinkStiffness);
					J_weighted += link_stiffness * N_row * N_row.transpose();
					Jt_err_weighted += link_stiffness * d_scale * N_row;
				}
			}
#endif

			Eigen::VectorXd y = J_weighted.lu().solve(Jt_err_weighted);
			Eigen::VectorXd x = N * y;
			cKinTree::ApplyStep(joint_desc, x, out_pose);
			
			bool is_last = p == max_priority;

			if (!is_last)
			{
				Eigen::MatrixXd cons_mat = Eigen::MatrixXd(num_valid_cons, cons_desc.cols());
				int r = 0;
				for (int c = 0; c < num_cons; ++c)
				{
					const tConsDesc& curr_cons = cons_desc.row(c);
					int curr_priority = static_cast<int>(curr_cons(eConsDescPriority));

					if (curr_priority == p)
					{
						cons_mat.row(r) = curr_cons;
						++r;
					}
				}

				J = BuildJacob(joint_desc, out_pose, cons_mat);
				J = J * N;

				Eigen::MatrixXd curr_N = BuildKernel(J);
				if (curr_N.cols() == 0)
				{
					break;
				}

				N = N * curr_N;
			}
		}
	}
}

Eigen::VectorXd cIKSolver::BuildErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const Eigen::MatrixXd& cons_mat)
{
	int cons_dim = CountConsDim(cons_mat);
	int num_cons = static_cast<int>(cons_mat.rows());
	Eigen::VectorXd err(cons_dim);

	int r = 0;
	for (int c = 0; c < num_cons; ++c)
	{
		const tConsDesc& cons_desc = cons_mat.row(c);

		Eigen::VectorXd curr_err = BuildErr(joint_mat, pose, cons_desc);
		int curr_dim = static_cast<int>(curr_err.rows());

		err.block(r, 0, curr_dim, 1) = curr_err;
		r += curr_dim;
	}

	return err;
}

Eigen::VectorXd cIKSolver::BuildErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc)
{
	const double clamp_dist = std::numeric_limits<double>::infinity();
	return BuildErr(joint_mat, pose, cons_desc, clamp_dist);
}

Eigen::VectorXd cIKSolver::BuildErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc, double clamp_dist)
{
	Eigen::VectorXd err;
	int cons_type = static_cast<int>(cons_desc(eConsDescType));
	switch (cons_type)
	{
		case eConsTypePos:
			err = BuildConsPosErr(joint_mat, pose, cons_desc, clamp_dist);
			break;
		case eConsTypePosX:
			err = BuildConsPosXErr(joint_mat, pose, cons_desc, clamp_dist);
			break;
		case eConsTypePosY:
			err = BuildConsPosYErr(joint_mat, pose, cons_desc, clamp_dist);
			break;
		case eConsTypeTheta:
			err = BuildConsThetaErr(joint_mat, pose, cons_desc);
			break;
		case eConsTypeThetaWorld:
			err = BuildConsThetaWorldErr(joint_mat, pose, cons_desc);
			break;
		default:
			assert(false); // unsupported constraint
			break;
	}
	return err;
}

Eigen::VectorXd cIKSolver::BuildConsPosErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc, double clamp_dist)
{
	assert(static_cast<int>(cons_desc(eConsDescType)) == eConsTypePos);

	int parent_id = static_cast<int>(cons_desc(eConsDescParam0));
	tVector attach_pt = tVector(cons_desc(eConsDescParam1), cons_desc(eConsDescParam2), 0.f, 0.f);
	tVector target_pos = tVector(cons_desc(eConsDescParam3), cons_desc(eConsDescParam4), 0.f, 0.f);

	tVector end_pos = cKinTree::LocalToWorldPos(joint_mat, pose, parent_id, attach_pt);
	tVector delta = target_pos - end_pos;

	ClampMag(delta, clamp_dist);

	Eigen::Vector2d err = Eigen::Vector2d(delta[0], delta[1]);
	return err;
}

Eigen::VectorXd cIKSolver::BuildConsPosXErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc, double clamp_dist)
{
	assert(static_cast<int>(cons_desc(eConsDescType)) == eConsTypePosX);

	int parent_id = static_cast<int>(cons_desc(eConsDescParam0));
	tVector attach_pt = tVector(cons_desc(eConsDescParam1), cons_desc(eConsDescParam2), 0.f, 0.f);
	tVector target_pos = tVector(cons_desc(eConsDescParam3), cons_desc(eConsDescParam4), 0.f, 0.f);

	tVector end_pos = cKinTree::LocalToWorldPos(joint_mat, pose, parent_id, attach_pt);
	tVector delta = target_pos - end_pos;

	ClampMag(delta, clamp_dist);

	Eigen::VectorXd err = Eigen::VectorXd(1);
	err(0) = delta[0];
	return err;
}

Eigen::VectorXd cIKSolver::BuildConsPosYErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc, double clamp_dist)
{
	assert(static_cast<int>(cons_desc(eConsDescType)) == eConsTypePosY);

	int parent_id = static_cast<int>(cons_desc(eConsDescParam0));
	tVector attach_pt = tVector(cons_desc(eConsDescParam1), cons_desc(eConsDescParam2), 0.f, 0.f);
	tVector target_pos = tVector(cons_desc(eConsDescParam3), cons_desc(eConsDescParam4), 0.f, 0.f);

	tVector end_pos = cKinTree::LocalToWorldPos(joint_mat, pose, parent_id, attach_pt);
	tVector delta = target_pos - end_pos;

	ClampMag(delta, clamp_dist);

	Eigen::VectorXd err = Eigen::VectorXd(1);
	err(0) = delta[1];
	return err;
}

Eigen::VectorXd cIKSolver::BuildConsThetaErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc)
{
	assert(static_cast<int>(cons_desc(eConsDescType)) == eConsTypeTheta);

	int joint_id = static_cast<int>(cons_desc(eConsDescParam0));
	double tar_theta = cons_desc(eConsDescParam1);
	double theta = cKinTree::GetJointTheta(joint_mat, pose, joint_id);
	double delta = tar_theta - theta;

	Eigen::VectorXd err = Eigen::VectorXd(1);
	err(0) = delta;
	return err;
}

Eigen::VectorXd cIKSolver::BuildConsThetaWorldErr(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc)
{
	assert(static_cast<int>(cons_desc(eConsDescType)) == eConsTypeThetaWorld);

	int joint_id = static_cast<int>(cons_desc(eConsDescParam0));
	double tar_theta = cons_desc(eConsDescParam1);

	double theta;
	tVector axis;
	cKinTree::CalcJointWorldTheta(joint_mat, pose, joint_id, axis, theta);

	double delta = tar_theta - theta;

	Eigen::VectorXd err = Eigen::VectorXd(1);
	err(0) = delta;
	return err;
}


Eigen::MatrixXd cIKSolver::BuildKernel(const Eigen::MatrixXd& mat)
{
	const double threshold = 0.0001f;
	Eigen::JacobiSVD<Eigen::MatrixXd> svd_new(mat, Eigen::ComputeFullV | Eigen::ComputeFullU);
	const auto& V_new = svd_new.matrixV();
	const auto& s_new = svd_new.singularValues();

	int start_null = 0;
	for (int i = 0; i < s_new.size(); ++i)
	{
		double val = s_new(i);
		if (std::abs(val) > threshold)
		{
			++start_null;
		}
	}
	Eigen::MatrixXd N = V_new.block(0, start_null, V_new.rows(), V_new.cols() - start_null);
	//Eigen::MatrixXd N = mat.fullPivLu().kernel();
	return N;
}

Eigen::MatrixXd cIKSolver::BuildJacob(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const Eigen::MatrixXd& cons_mat)
{
	int num_dof = cKinTree::GetNumDof(joint_mat);
	int cons_dim = CountConsDim(cons_mat);
	int num_cons = static_cast<int>(cons_mat.rows());

	Eigen::MatrixXd J = Eigen::MatrixXd(cons_dim, num_dof);
	int r = 0;
	for (int c = 0; c < num_cons; ++c)
	{
		const tConsDesc& curr_cons = cons_mat.row(c);
		Eigen::MatrixXd curr_J = BuildJacob(joint_mat, pose, curr_cons);
		int curr_dim = static_cast<int>(curr_J.rows());
		J.block(r, 0, curr_dim, num_dof) = curr_J;
		r += curr_dim;
	}
	return J;
}

Eigen::MatrixXd cIKSolver::BuildJacob(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc)
{
	Eigen::MatrixXd J;
	eConsDesc cons_type = static_cast<eConsDesc>(static_cast<int>(cons_desc(eConsDescType)));
	switch (cons_type)
	{
		case eConsTypePos:
			J = BuildConsPosJacob(joint_mat, pose, cons_desc);
			break;
		case eConsTypePosX:
			J = BuildConsPosXJacob(joint_mat, pose, cons_desc);
			break;
		case eConsTypePosY:
			J = BuildConsPosYJacob(joint_mat, pose, cons_desc);
			break;
		case eConsTypeTheta:
			J = BuildConsThetaJacob(joint_mat, pose, cons_desc);
			break;
		case eConsTypeThetaWorld:
			J = BuildConsThetaWorldJacob(joint_mat, pose, cons_desc);
			break;
		default:
			assert(false); // unsupported constraint
			break;
	}
	return J;
}

Eigen::MatrixXd cIKSolver::BuildConsPosJacob(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc)
{
	assert(static_cast<int>(cons_desc(eConsDescType)) == eConsTypePos);

	int num_joints = static_cast<int>(joint_mat.rows());
	int parent_id = static_cast<int>(cons_desc(eConsDescParam0));
	tVector attach_pt = tVector(cons_desc(eConsDescParam1), cons_desc(eConsDescParam2), 0.f, 0.f);
	tVector end_pos = cKinTree::LocalToWorldPos(joint_mat, pose, parent_id, attach_pt);

	const Eigen::Vector3d rot_axis = Eigen::Vector3d(0, 0, 1);

	int num_dof = cKinTree::GetNumDof(joint_mat);
	Eigen::MatrixXd J = Eigen::MatrixXd(gPosDims, num_dof);
	J.setZero();

	for (int i = 0; i < gPosDims; ++i)
	{
		J(i, i) = 1;
	}
	 
	int curr_id = parent_id;
	while (true)
	{
		tVector joint_pos = cKinTree::CalcJointWorldPos(joint_mat, pose, curr_id);
		tVector delta = end_pos - joint_pos;

		Eigen::Vector3d tangent = rot_axis.cross(Eigen::Vector3d(delta(0), delta(1), delta(2)));
		int curr_parent_id = cKinTree::GetParent(joint_mat, curr_id);

		for (int i = 0; i < gPosDims; ++i)
		{
			J(i, gPosDims + curr_id) = tangent(i);
		}
		if (curr_parent_id == cKinTree::gInvalidJointID)
		{
			// no scaling for root
			break;
		}
		else
		{
#if !defined(DISABLE_LINK_SCALE)
			double attach_x = joint_desc(curr_id, cKinTree::eJointDescAttachX);
			double attach_y = joint_desc(curr_id, cKinTree::eJointDescAttachY);
			double attach_z = joint_desc(curr_id, cKinTree::eJointDescAttachZ);

			double parent_world_theta = cKinTree::CalcJointWorldTheta(joint_desc, curr_parent_id);
			double world_attach_x = std::cos(parent_world_theta) * attach_x - std::sin(parent_world_theta) * attach_y;
			double world_attach_y = std::sin(parent_world_theta) * attach_x + std::cos(parent_world_theta) * attach_y;
			double world_attach_z = attach_z; // hack ignoring z, do this properly

			J(0, gPosDims + num_joints + curr_id) = world_attach_x;
			J(1, gPosDims + num_joints + curr_id) = world_attach_y;
#endif
			curr_id = curr_parent_id;
		}
	}

	return J;
}

Eigen::MatrixXd cIKSolver::BuildConsPosXJacob(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc)
{
	assert(static_cast<int>(cons_desc(eConsDescType)) == eConsTypePosX);

	int num_joints = static_cast<int>(joint_mat.rows());
	int parent_id = static_cast<int>(cons_desc(eConsDescParam0));
	tVector attach_pt = tVector(cons_desc(eConsDescParam1), cons_desc(eConsDescParam2), 0.f, 0.f);
	tVector end_pos = cKinTree::LocalToWorldPos(joint_mat, pose, parent_id, attach_pt);

	const Eigen::Vector3d rot_axis = Eigen::Vector3d(0, 0, 1);

	int num_dof = cKinTree::GetNumDof(joint_mat);
	int cons_dim = GetConsDim(cons_desc);
	Eigen::MatrixXd J = Eigen::MatrixXd(cons_dim, num_dof);
	J.setZero();

	J(0, 0) = 1;

	int curr_id = parent_id;
	while (true)
	{
		tVector joint_pos = cKinTree::CalcJointWorldPos(joint_mat, pose, curr_id);
		tVector delta = end_pos - joint_pos;

		Eigen::Vector3d tangent = rot_axis.cross(Eigen::Vector3d(delta(0), delta(1), delta(2)));
		int curr_parent_id = cKinTree::GetParent(joint_mat, curr_id);

		J(0, gPosDims + curr_id) = tangent(0);

		if (curr_parent_id == cKinTree::gInvalidJointID)
		{
			// no scaling for root
			break;
		}
		else
		{
#if !defined(DISABLE_LINK_SCALE)
			double attach_x = joint_desc(curr_id, cKinTree::eJointDescAttachX);
			double attach_y = joint_desc(curr_id, cKinTree::eJointDescAttachY);
			double attach_z = joint_desc(curr_id, cKinTree::eJointDescAttachZ);

			double parent_world_theta = cKinTree::CalcJointWorldTheta(joint_desc, curr_parent_id);
			double world_attach_x = std::cos(parent_world_theta) * attach_x - std::sin(parent_world_theta) * attach_y;
			
			J(0, gPosDims + num_joints + curr_id) = world_attach_x;
#endif
			curr_id = curr_parent_id;
		}
	}

	return J;
}

Eigen::MatrixXd cIKSolver::BuildConsPosYJacob(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc)
{
	assert(static_cast<int>(cons_desc(eConsDescType)) == eConsTypePosY);

	int num_joints = static_cast<int>(joint_mat.rows());
	int parent_id = static_cast<int>(cons_desc(eConsDescParam0));
	tVector attach_pt = tVector(cons_desc(eConsDescParam1), cons_desc(eConsDescParam2), 0.f, 0.f);
	tVector end_pos = cKinTree::LocalToWorldPos(joint_mat, pose, parent_id, attach_pt);

	const Eigen::Vector3d rot_axis = Eigen::Vector3d(0, 0, 1);

	int num_dof = cKinTree::GetNumDof(joint_mat);
	int cons_dim = GetConsDim(cons_desc);
	Eigen::MatrixXd J = Eigen::MatrixXd(cons_dim, num_dof);
	J.setZero();

	J(0, 1) = 1;

	int curr_id = parent_id;
	while (true)
	{
		tVector joint_pos = cKinTree::CalcJointWorldPos(joint_mat, pose, curr_id);
		tVector delta = end_pos - joint_pos;

		Eigen::Vector3d tangent = rot_axis.cross(Eigen::Vector3d(delta(0), delta(1), delta(2)));
		int curr_parent_id = cKinTree::GetParent(joint_mat, curr_id);

		J(0, gPosDims + curr_id) = tangent(1);

		if (curr_parent_id == cKinTree::gInvalidJointID)
		{
			// no scaling for root
			break;
		}
		else
		{
#if !defined(DISABLE_LINK_SCALE)
			double attach_x = joint_desc(curr_id, cKinTree::eJointDescAttachX);
			double attach_y = joint_desc(curr_id, cKinTree::eJointDescAttachY);
			double attach_z = joint_desc(curr_id, cKinTree::eJointDescAttachZ);

			double parent_world_theta = cKinTree::CalcJointWorldTheta(joint_desc, curr_parent_id);
			double world_attach_y = std::sin(parent_world_theta) * attach_x + std::cos(parent_world_theta) * attach_y;

			J(0, gPosDims + num_joints + curr_id) = world_attach_y;
#endif
			curr_id = curr_parent_id;
		}
	}

	return J;
}

Eigen::MatrixXd cIKSolver::BuildConsThetaJacob(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc)
{
	assert(static_cast<int>(cons_desc(eConsDescType)) == eConsTypeTheta);

	int num_joints = static_cast<int>(joint_mat.rows());
	int joint_id = static_cast<int>(cons_desc(eConsDescParam0));

	int num_dof = cKinTree::GetNumDof(joint_mat);
	int cons_dim = GetConsDim(cons_desc);
	Eigen::MatrixXd J = Eigen::MatrixXd(cons_dim, num_dof);
	J.setZero();

	J(0, gPosDims + joint_id) = 1;

	return J;
}

Eigen::MatrixXd cIKSolver::BuildConsThetaWorldJacob(const Eigen::MatrixXd& joint_mat, const Eigen::VectorXd& pose, const tConsDesc& cons_desc)
{
	assert(static_cast<int>(cons_desc(eConsDescType)) == eConsTypeThetaWorld);

	int num_joints = static_cast<int>(joint_mat.rows());
	int joint_id = static_cast<int>(cons_desc(eConsDescParam0));

	int num_dof = cKinTree::GetNumDof(joint_mat);
	int cons_dim = GetConsDim(cons_desc);
	Eigen::MatrixXd J = Eigen::MatrixXd(cons_dim, num_dof);
	J.setZero();

	int curr_id = joint_id;
	while (true)
	{
		J(0, gPosDims + curr_id) = 1;

		int curr_parent_id = cKinTree::GetParent(joint_mat, curr_id);
		if (curr_parent_id == cKinTree::gInvalidJointID)
		{
			break;
		}
		curr_id = curr_parent_id;
	}

	return J;
}

int cIKSolver::CountConsDim(const Eigen::MatrixXd& cons_mat)
{
	assert(cons_mat.cols() == eConsDescMax);
	int count = 0;
	int num_cons = static_cast<int>(cons_mat.rows());
	for (int c = 0; c < num_cons; ++c)
	{
		const tConsDesc& curr_cons = cons_mat.row(c);
		int curr_dim = GetConsDim(curr_cons);
		count += curr_dim;
	}
	return count;
}

int cIKSolver::GetConsDim(const tConsDesc& cons_desc)
{
	eConsType cons_type = static_cast<eConsType>(static_cast<int>(cons_desc(eConsDescType)));
	switch (cons_type)
	{
		case eConsTypePos:
			return gPosDims;
		case eConsTypePosX:
		case eConsTypePosY:
		case eConsTypeTheta:
		case eConsTypeThetaWorld:
			return 1;
		default:
			assert(false); // unsupported constraint
			break;
	}
	return 0;
}

void cIKSolver::ClampMag(tVector& vec, double max_d)
{
	double len_sq = vec.squaredNorm();
	if (len_sq > max_d * max_d)
	{
		vec = (max_d / std::sqrt(len_sq)) * vec;
	}
}
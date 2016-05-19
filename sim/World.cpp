#include "World.h"

#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "sim/SimObj.h"
#include "sim/SimBox.h"
#include "sim/SimCapsule.h"
#include "sim/SimPlane.h"
#include "sim/Joint.h"

cWorld::tParams::tParams()
{
	mNumSubsteps = 1;
	mScale = 1;
	mGravity = gGravity;
}

cWorld::tJointParams::tJointParams()
{
	mType = eJointTypeHinge;
	mAnchor0 = tVector::Zero();
	mAnchor1 = tVector::Zero();
	mAnchorAxis1 = tVector(0, 0, 1, 0);
	mAnchorTheta1 = 0;
	mAxis = tVector(0, 1, 0, 0);
	mEnableAdjacentCollision = true;
	mLimLow = 1; // low > high -> no limit
	mLimHigh = 0;
	mRefTheta = 0;
}

cWorld::tConstraintHandle::tConstraintHandle()
{
	mCons = nullptr;
}

bool cWorld::tConstraintHandle::IsValid() const
{
	return mCons != nullptr;
}

void cWorld::tConstraintHandle::Clear()
{
	delete mCons;
	mCons = nullptr;
}

cWorld::cWorld()
	: mContactManager(*this)
{
	mLinearDamping = 0;
	mAngularDamping = 0;
}

cWorld::~cWorld()
{
	ClearConstraints();
}

void cWorld::Init(const tParams& params)
{
	mParams = params;

	mBroadPhase = std::unique_ptr<btBroadphaseInterface>(new btDbvtBroadphase());
	mCollisionConfig = std::unique_ptr<btDefaultCollisionConfiguration>(new btDefaultCollisionConfiguration());
	mCollisionDispatcher = std::unique_ptr<btCollisionDispatcher>(new btCollisionDispatcher(mCollisionConfig.get()));
	btGImpactCollisionAlgorithm::registerAlgorithm(mCollisionDispatcher.get());

	mSolver = std::unique_ptr<btSequentialImpulseConstraintSolver>(new btSequentialImpulseConstraintSolver());
	mSimWorld = std::unique_ptr<btDiscreteDynamicsWorld>(new btDiscreteDynamicsWorld(mCollisionDispatcher.get(),
		mBroadPhase.get(), mSolver.get(), mCollisionConfig.get()));
	SetGravity(params.mGravity);

	mContactManager.Init();
	mPerturbManager.Clear();
}

void cWorld::Reset()
{
	mContactManager.Reset();
	mPerturbManager.Clear();

	mSimWorld->clearForces();
	mSolver->reset();
	mBroadPhase->resetPool(mCollisionDispatcher.get());

	btOverlappingPairCache* pair_cache = mSimWorld->getBroadphase()->getOverlappingPairCache();
	btBroadphasePairArray& pair_array = pair_cache->getOverlappingPairArray();
	for (int i = 0; i < pair_array.size(); ++i)
	{
		pair_cache->cleanOverlappingPair(pair_array[i], mSimWorld->getDispatcher());
	}
}

void cWorld::Update(double time_elapsed)
{
	time_elapsed = std::max(0.0, time_elapsed);
	mPerturbManager.Update(time_elapsed);

	btScalar time_step = static_cast<btScalar>(time_elapsed);
	mSimWorld->stepSimulation(time_step, mParams.mNumSubsteps, time_step / mParams.mNumSubsteps);

	mContactManager.Update();
}

void cWorld::AddObject(cSimObj& obj)
{
	const std::unique_ptr<btRigidBody>& body = obj.GetRigidBody();

	short col_group = obj.GetColGroup();
	short col_mask = obj.GetColMask();
	col_mask |= cContactManager::gFlagRayTest;

	body->setDamping(static_cast<btScalar>(mLinearDamping), static_cast<btScalar>(mAngularDamping));
	mSimWorld->addRigidBody(body.get(), col_group, col_mask);
}

void cWorld::RemoveObject(cSimObj& obj)
{
	/*
	if (obj.GetRigidBody() && obj.GetRigidBody()->getMotionState())
	{
	delete obj.GetRigidBody().get()->getMotionState();
	}*/
	// mSimWorld->removeCollisionObject(obj.GetRigidBody().get());
	mSimWorld->removeRigidBody(obj.GetRigidBody().get());
	// std::cout << "Number of colission objects" << mSimWorld->m_collisionShapes.size() << std::endl;
	// obj.GetRigidBody().reset();
}

cWorld::tConstraintHandle cWorld::AddJoint(cSimObj* obj, const tJointParams& params)
{
	return AddJoint(obj, nullptr, params);
}

cWorld::tConstraintHandle cWorld::AddJoint(cSimObj* obj0, cSimObj* obj1, const tJointParams& params)
{
	tConstraintHandle handle;
	switch (params.mType)
	{
	case eJointTypeHinge:
		handle = AddHingeConstraint(obj0, obj1, params);
		break;
	case eJointTypePrismatic:
		handle = AddPrismaticConstraint(obj0, obj1, params);
		break;
	default:
		assert(false);
		printf("Unsupported constraint type\n");
		break;
	}

	return handle;
}

void cWorld::ConstrainPlane(cSimObj* obj, ePlaneCons plane_cons)
{
	tVector linear_factor;
	tVector angular_factor;
	BuildConsFactor(plane_cons, linear_factor, angular_factor);
	Constrain(obj, linear_factor, angular_factor);
}

void cWorld::Constrain(cSimObj* obj, const tVector& linear_factor, const tVector& angular_factor)
{
	auto& body = obj->GetRigidBody();
	body->setLinearFactor(btVector3(static_cast<btScalar>(linear_factor[0]),
		static_cast<btScalar>(linear_factor[1]),
		static_cast<btScalar>(linear_factor[2])));
	body->setAngularFactor(btVector3(static_cast<btScalar>(angular_factor[0]),
		static_cast<btScalar>(angular_factor[1]),
		static_cast<btScalar>(angular_factor[2])));
}

void cWorld::RemoveConstraint(tConstraintHandle& handle)
{
	if (handle.IsValid())
	{
		mSimWorld->removeConstraint(handle.mCons);
	}
	handle.Clear();
}

void cWorld::BuildConsFactor(ePlaneCons plane_cons, tVector& out_linear_factor, tVector& out_angular_factor)
{
	out_linear_factor = tVector::Ones();
	out_angular_factor = tVector::Ones();
	switch (plane_cons)
	{
	case ePlaneConsNone:
		break;
	case ePlaneConsXY:
		out_linear_factor = tVector(1, 1, 0, 0);
		out_angular_factor = tVector(0, 0, 1, 0);
		break;
	default:
		assert(false); // unsupported constraint
	}
}

void cWorld::SetGravity(const tVector& gravity)
{
	double scale = GetScale();
	mSimWorld->setGravity(btVector3(static_cast<btScalar>(gravity[0] * scale),
		static_cast<btScalar>(gravity[1] * scale),
		static_cast<btScalar>(gravity[2] * scale)));
}

cContactManager::tContactHandle cWorld::RegisterContact(int contact_flags, int filter_flags)
{
	return mContactManager.RegisterContact(contact_flags, filter_flags);
}

void cWorld::UpdateContact(const cContactManager::tContactHandle& handle)
{
	mContactManager.UpdateContact(handle);
}

tVector cWorld::GetContactPt(const cContactManager::tContactHandle& handle) const
{
	return mContactManager.GetContactPt(handle);
}

bool cWorld::IsInContact(const cContactManager::tContactHandle& handle) const
{
	return mContactManager.IsInContact(handle);
}

void cWorld::RayTest(const tVector& beg, const tVector& end, tRayTestResults& results) const
{
	btScalar scale = static_cast<btScalar>(GetScale());
	btVector3 bt_beg = scale * btVector3(static_cast<btScalar>(beg[0]),
		static_cast<btScalar>(beg[1]),
		static_cast<btScalar>(beg[2]));
	btVector3 bt_end = scale * btVector3(static_cast<btScalar>(end[0]),
		static_cast<btScalar>(end[1]),
		static_cast<btScalar>(end[2]));
	btCollisionWorld::ClosestRayResultCallback ray_callback(bt_beg, bt_end);

	mSimWorld->rayTest(bt_beg, bt_end, ray_callback);

	results.clear();
	if (ray_callback.hasHit())
	{
		auto& obj = ray_callback.m_collisionObject;
		const auto& hit_pt = ray_callback.m_hitPointWorld;
		tRayTestResult result;
		result.mObj = static_cast<cSimObj*>(obj->getUserPointer());
		result.mHitPos = tVector(hit_pt[0], hit_pt[1], hit_pt[2], 0) / scale;

		results.push_back(result);
	}
}

void cWorld::AddPerturb(const tPerturb& perturb)
{
	mPerturbManager.AddPerturb(perturb);
}

const cPerturbManager& cWorld::GetPerturbManager() const
{
	return mPerturbManager;
}

tVector cWorld::GetGravity() const
{
	double scale = GetScale();
	btVector3 bt_gravity = mSimWorld->getGravity();
	return tVector(bt_gravity[0], bt_gravity[1], bt_gravity[2], 0) / scale;
}

double cWorld::GetScale() const
{
	return mParams.mScale;
}

void cWorld::SetLinearDamping(double damping)
{
	mLinearDamping = damping;
}

void cWorld::SetAngularDamping(double damping)
{
	mAngularDamping = damping;
}

tVector cWorld::GetPos(const cSimObj* obj) const
{
	auto& body = obj->GetRigidBody();
	btTransform trans = body->getWorldTransform();
	btVector3 origin = trans.getOrigin();
	tVector pos = tVector(origin[0], origin[1], origin[2], 0);
	pos /= GetScale();
	return pos;
}

void cWorld::SetPos(const tVector& pos, cSimObj* out_obj) const
{
	auto& body = out_obj->GetRigidBody();
	btTransform trans = body->getWorldTransform();

	btScalar scale = static_cast<btScalar>(GetScale());
	trans.setOrigin(scale * btVector3(static_cast<btScalar>(pos[0]),
		static_cast<btScalar>(pos[1]),
		static_cast<btScalar>(pos[2])));
	body->setWorldTransform(trans);
	body->activate();
}

tVector cWorld::GetLinearVelocity(const cSimObj* obj) const
{
	auto& body = obj->GetRigidBody();
	const btVector3& bt_vel = body->getLinearVelocity();
	tVector vel = tVector(bt_vel[0], bt_vel[1], bt_vel[2], 0);
	vel /= GetScale();
	return vel;
}

tVector cWorld::GetLinearVelocity(const cSimObj* obj, const tVector& local_pos) const
{
	auto& body = obj->GetRigidBody();
	btScalar scale = static_cast<btScalar>(GetScale());
	tMatrix3 rot_mat = obj->GetLocalToWorldRotMat();
	tVector rel_pos = tVector::Zero();
	rel_pos.segment(0, 3) = rot_mat * local_pos.segment(0, 3);

	btVector3 bt_vel = body->getVelocityInLocalPoint(scale * btVector3(static_cast<btScalar>(rel_pos[0]),
		static_cast<btScalar>(rel_pos[1]),
		static_cast<btScalar>(rel_pos[2])));
	tVector vel = tVector(bt_vel[0], bt_vel[1], bt_vel[2], 0);
	vel /= GetScale();
	return vel;
}

void cWorld::SetLinearVelocity(const tVector& vel, cSimObj* out_obj) const
{
	auto& body = out_obj->GetRigidBody();
	btScalar scale = static_cast<btScalar>(GetScale());
	body->setLinearVelocity(scale * btVector3(static_cast<btScalar>(vel[0]),
		static_cast<btScalar>(vel[1]),
		static_cast<btScalar>(vel[2])));
	body->activate();
}

tVector cWorld::GetAngularVelocity(const cSimObj* obj) const
{
	auto& body = obj->GetRigidBody();
	const btVector3& vel = body->getAngularVelocity();
	return tVector(vel[0], vel[1], vel[2], 0);
}

void cWorld::SetAngularVelocity(const tVector& vel, const cSimObj* obj) const
{
	auto& body = obj->GetRigidBody();
	body->setAngularVelocity(btVector3(static_cast<btScalar>(vel[0]), static_cast<btScalar>(vel[1]),
		static_cast<btScalar>(vel[2])));
	body->activate();
}

tMatrix cWorld::GetWorldTransform(const cSimObj* obj) const
{
	tMatrix trans = GetWorldTransformAux(obj);
	return trans;
}

tMatrix cWorld::GetLocalTransform(const cSimObj* obj) const
{
	tMatrix trans = GetWorldTransformAux(obj);
	trans = cMathUtil::InvRigidMat(trans);
	return trans;
}

void cWorld::GetRotation(const cSimObj* obj, tVector& out_axis, double& out_theta) const
{
	btQuaternion q = GetRotQuaternion(obj);
	btVector3 axis = q.getAxis();

	out_theta = q.getAngle();
	out_axis[0] = axis[0];
	out_axis[1] = axis[1];
	out_axis[2] = axis[2];
	out_axis[3] = 0;
}

btQuaternion cWorld::GetRotQuaternion(const cSimObj* obj) const
{
	auto& body = obj->GetRigidBody();
	btTransform trans = body->getWorldTransform();
	btQuaternion q = trans.getRotation();
	return q;
}

tMatrix cWorld::GetWorldTransformAux(const cSimObj* obj) const
{
	auto& body = obj->GetRigidBody();
	const btTransform& bt_trans = body->getWorldTransform();
	const btMatrix3x3& basis = bt_trans.getBasis();
	const btVector3& origin = bt_trans.getOrigin();
	double scale = GetScale();

	tMatrix trans = tMatrix::Identity();
	for (int i = 0; i < 3; ++i)
	{
		auto curr_row = trans.row(i);
		auto bt_row = basis.getRow(i);
		for (int j = 0; j < 3; ++j)
		{
			curr_row[j] = bt_row[j];
		}
		curr_row[3] = origin[i] / scale;
	}
	return trans;
}

void cWorld::SetRotation(const tVector& axis, double theta, cSimObj* out_obj) const
{
	auto& body = out_obj->GetRigidBody();
	btTransform trans = body->getWorldTransform();
	double norm = axis.norm();

	btVector3 bt_axis = btVector3(static_cast<btScalar>(axis[0]), static_cast<btScalar>(axis[1]),
		static_cast<btScalar>(axis[2]));
	bt_axis.normalize();

	btQuaternion q;
	q.setRotation(bt_axis, static_cast<btScalar>(theta));

	trans.setRotation(q);
	body->setWorldTransform(trans);
}

void cWorld::CalcAABB(const cSimObj* obj, tVector& out_min, tVector& out_max) const
{
	auto& body = obj->GetRigidBody();
	btVector3 bt_min;
	btVector3 bt_max;
	body->getAabb(bt_min, bt_max);
	double scale = GetScale();

	out_min = tVector(bt_min[0], bt_min[1], bt_min[2], 0) / scale;
	out_max = tVector(bt_max[0], bt_max[1], bt_max[2], 0) / scale;
}

void cWorld::ApplyForce(const tVector& force, const tVector& local_pos, cSimObj* out_obj) const
{
	auto& body = out_obj->GetRigidBody();
	btVector3 bt_force = btVector3(static_cast<btScalar>(force[0]),
		static_cast<btScalar>(force[1]),
		static_cast<btScalar>(force[2]));

	btScalar scale = static_cast<btScalar>(GetScale());
	tMatrix3 rot_mat = out_obj->GetLocalToWorldRotMat();
	tVector rel_pos = tVector::Zero();
	rel_pos.segment(0, 3) = rot_mat * local_pos.segment(0, 3);

	btVector3 bt_pos = btVector3(static_cast<btScalar>(rel_pos[0]),
		static_cast<btScalar>(rel_pos[1]),
		static_cast<btScalar>(rel_pos[2]));

	bt_force *= scale;
	bt_pos *= scale;
	body->applyForce(bt_force, bt_pos);
}

void cWorld::ApplyTorque(const tVector& torque, cSimObj* out_obj) const
{
	auto& body = out_obj->GetRigidBody();
	btScalar scale = static_cast<btScalar>(GetScale());
	body->applyTorque(scale * scale * btVector3(static_cast<btScalar>(torque[0]),
		static_cast<btScalar>(torque[1]),
		static_cast<btScalar>(torque[2])));
}

std::unique_ptr<btBoxShape> cWorld::BuildBoxShape(const tVector& box_size) const
{
	btScalar scale = static_cast<btScalar>(GetScale());
	std::unique_ptr<btBoxShape> shape = std::unique_ptr<btBoxShape>(new btBoxShape(scale * btVector3(static_cast<btScalar>(box_size[0] * 0.5),
		static_cast<btScalar>(box_size[1] * 0.5),
		static_cast<btScalar>(box_size[2] * 0.5))));
	return shape;
}

std::unique_ptr<btCapsuleShape> cWorld::BuildCapsuleShape(double radius, double height) const
{
	btScalar scale = static_cast<btScalar>(GetScale());
	std::unique_ptr<btCapsuleShape> shape = std::unique_ptr<btCapsuleShape>(new btCapsuleShape(static_cast<btScalar>(scale * radius),
		static_cast<btScalar>(scale * height)));
	return shape;
}

std::unique_ptr<btStaticPlaneShape> cWorld::BuildPlaneShape(const tVector& normal, const tVector& origin) const
{
	btVector3 bt_normal = btVector3(static_cast<btScalar>(normal[0]),
		static_cast<btScalar>(normal[1]),
		static_cast<btScalar>(normal[2]));
	btVector3 bt_origin = btVector3(static_cast<btScalar>(origin[0]),
		static_cast<btScalar>(origin[1]),
		static_cast<btScalar>(origin[2]));
	bt_normal.normalize();
	double scale = GetScale();
	btScalar w = static_cast<btScalar>(scale * bt_normal.dot(bt_origin));

	std::unique_ptr<btStaticPlaneShape> shape = std::unique_ptr<btStaticPlaneShape>(new btStaticPlaneShape(bt_normal, w));
	return shape;
}

tVector cWorld::GetBoxSize(const cSimBox* box) const
{
	const btBoxShape* box_shape = reinterpret_cast<btBoxShape*>(box->GetCollisionShape().get());
	btVector3 half_len = box_shape->getHalfExtentsWithMargin();
	double scale = GetScale();
	return tVector(half_len[0] * 2, half_len[1] * 2, half_len[2] * 2, 0) / scale;
}

double cWorld::GetCapsuleHeight(const cSimCapsule* cap) const
{
	const btCapsuleShape* shape = reinterpret_cast<btCapsuleShape*>(cap->GetCollisionShape().get());
	double scale = GetScale();
	double h = shape->getHalfHeight() * 2;
	h /= scale;
	return h;
}

double cWorld::GetCapsuleRadius(const cSimCapsule* cap) const
{
	const btCapsuleShape* shape = reinterpret_cast<btCapsuleShape*>(cap->GetCollisionShape().get());
	double scale = GetScale();
	double r = shape->getRadius();
	r /= scale;
	return r;
}

tVector cWorld::GetPlaneCoeffs(const cSimPlane* plane) const
{
	const btStaticPlaneShape* shape = reinterpret_cast<btStaticPlaneShape*>(plane->GetCollisionShape().get());
	double scale = GetScale();
	btVector3 n = shape->getPlaneNormal();
	btScalar c = shape->getPlaneConstant();
	return tVector(n[0], n[1], n[2], c / scale);
}

void cWorld::CalcRotationHinge(const cJoint* joint, tVector& out_axis, double& out_theta) const
{
	btHingeConstraint* hinge = reinterpret_cast<btHingeConstraint*>(joint->GetConstraintHandle().mCons);

	double ref_theta = joint->GetRefTheta();
	out_theta = hinge->getHingeAngle();
	out_theta = -out_theta; // theta flipped
							// std::cout << "ref_theta " << ref_theta << std::endl;
	out_theta -= ref_theta;
	out_axis = joint->GetAxisRel();
}

double cWorld::CalcDisplacementPrismatic(const cJoint* joint) const
{
	btSliderConstraint* prismatic = reinterpret_cast<btSliderConstraint*>(joint->GetConstraintHandle().mCons);
	double scale = GetScale();
	double delta = prismatic->getLinearPos();
	delta = -delta;
	delta /= scale;
	return delta;
}

tVector cWorld::GetManifoldPtA(const btManifoldPoint& manifold_pt) const
{
	double scale = GetScale();
	const btVector3& contact_pt = manifold_pt.getPositionWorldOnA();
	return tVector(contact_pt[0], contact_pt[1], contact_pt[2], 0) / scale;
}

tVector cWorld::GetManifoldPtB(const btManifoldPoint& manifold_pt) const
{
	double scale = GetScale();
	const btVector3& contact_pt = manifold_pt.getPositionWorldOnB();
	return tVector(contact_pt[0], contact_pt[1], contact_pt[2], 0) / scale;
}

std::unique_ptr<btDiscreteDynamicsWorld>& cWorld::GetInternalWorld()
{
	return mSimWorld;
}


void cWorld::ClearConstraints()
{
	for (int i = GetNumConstriants() - 1; i >= 0; i--)
	{
		btTypedConstraint* constraint = mSimWorld->getConstraint(i);
		mSimWorld->removeConstraint(constraint);
		delete constraint;
	}
}

int cWorld::GetNumConstriants() const
{
	return static_cast<int>(mSimWorld->getNumConstraints());
}

cWorld::tConstraintHandle cWorld::AddHingeConstraint(cSimObj* obj0, cSimObj* obj1, const tJointParams& params)
{
	btScalar scale = static_cast<btScalar>(GetScale());
	btVector3 anchor0 = scale * btVector3(static_cast<btScalar>(params.mAnchor0[0]),
		static_cast<btScalar>(params.mAnchor0[1]),
		static_cast<btScalar>(params.mAnchor0[2]));
	btVector3 anchor1 = scale * btVector3(static_cast<btScalar>(params.mAnchor1[0]),
		static_cast<btScalar>(params.mAnchor1[1]),
		static_cast<btScalar>(params.mAnchor1[2]));
	btVector3 axis = btVector3(static_cast<btScalar>(params.mAxis[0]),
		static_cast<btScalar>(params.mAxis[1]),
		static_cast<btScalar>(params.mAxis[2]));

	btHingeConstraint* cons = nullptr;
	if (obj1 != nullptr)
	{
		cons = new btHingeConstraint(*(obj0->GetRigidBody()), *(obj1->GetRigidBody()),
			anchor0, anchor1, axis, axis);
	}
	else
	{
		cons = new btHingeConstraint(*(obj0->GetRigidBody()), anchor0, axis);
	}

	// bullet limits are flipped
	cons->setLimit(-static_cast<btScalar>(params.mLimHigh), -static_cast<btScalar>(params.mLimLow));
	mSimWorld->addConstraint(cons, !params.mEnableAdjacentCollision);

	tConstraintHandle handle;
	handle.mCons = cons;
	return handle;
}

cWorld::tConstraintHandle cWorld::AddPrismaticConstraint(cSimObj* obj0, cSimObj* obj1, const tJointParams& params)
{
	btTransform anchor0_t;
	anchor0_t.setIdentity();
	btScalar scale = static_cast<btScalar>(GetScale());
	btVector3 anchor0 = scale * btVector3(static_cast<btScalar>(params.mAnchor0[0]),
		static_cast<btScalar>(params.mAnchor0[1]),
		static_cast<btScalar>(params.mAnchor0[2]));
	anchor0_t.setOrigin(anchor0);
	anchor0_t.getBasis().setEulerZYX(0.f, 0.f, static_cast<btScalar>(M_PI_2));
	btTransform anchor1_t;
	anchor1_t.setIdentity();
	btVector3 anchor1 = scale * btVector3(static_cast<btScalar>(params.mAnchor1[0]),
		static_cast<btScalar>(params.mAnchor1[1]),
		static_cast<btScalar>(params.mAnchor1[2]));
	anchor1_t.setOrigin(anchor1);
	anchor1_t.getBasis().setEulerZYX(0.f, 0.f, static_cast<btScalar>(M_PI_2));
	btVector3 axis = btVector3(static_cast<btScalar>(params.mAxis[0]),
		static_cast<btScalar>(params.mAxis[1]),
		static_cast<btScalar>(params.mAxis[2]));

	btSliderConstraint* cons = nullptr;
	if (obj1 != nullptr)
	{
		cons = new btSliderConstraint(*(obj0->GetRigidBody()), *(obj1->GetRigidBody()),
			anchor0_t, anchor1_t, true);
	}

	cons->setLowerLinLimit(static_cast<btScalar>(scale * params.mLimLow));
	cons->setUpperLinLimit(static_cast<btScalar>(scale * params.mLimHigh));
	cons->setLowerAngLimit(0.0f);
	cons->setUpperAngLimit(0.0f);
	mSimWorld->addConstraint(cons, !params.mEnableAdjacentCollision);

	tConstraintHandle handle;
	handle.mCons = cons;
	return handle;
}

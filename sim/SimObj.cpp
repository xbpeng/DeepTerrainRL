#include "SimObj.h"

cSimObj::cSimObj()
	: mWorld(nullptr)
{
	mType = eTypeDynamic;
	mColGroup = cContactManager::gFlagAll;
	mColMask = cContactManager::gFlagAll;
}

cSimObj::~cSimObj()
{
	RemoveFromWorld();
}

tVector cSimObj::GetPos() const
{
	return mWorld->GetPos(this);
}

void cSimObj::SetPos(const tVector& pos)
{
	mWorld->SetPos(pos, this);
}

void cSimObj::GetRotation(tVector& out_axis, double& out_theta) const
{
	mWorld->GetRotation(this, out_axis, out_theta);
}

btQuaternion cSimObj::GetRotQuaternion() const
{
	return mWorld->GetRotQuaternion(this);
}

void cSimObj::SetRotation(const tVector& axis, double theta)
{
	mWorld->SetRotation(axis, theta, this);
}

tVector cSimObj::GetLinearVelocity() const
{
	return mWorld->GetLinearVelocity(this);
}

tVector cSimObj::GetLinearVelocity(const tVector& local_pos) const
{
	return mWorld->GetLinearVelocity(this, local_pos);
}

void cSimObj::SetLinearVelocity(const tVector& vel)
{
	mWorld->SetLinearVelocity(vel, this);
}

tVector cSimObj::GetAngularVelocity() const
{
	return mWorld->GetAngularVelocity(this);
}

void cSimObj::SetAngularVelocity(const tVector& vel)
{
	mWorld->SetAngularVelocity(vel, this);
}

tMatrix cSimObj::GetWorldTransform() const
{
	return mWorld->GetWorldTransform(this);
}

tMatrix cSimObj::GetLocalTransform() const
{
	return mWorld->GetLocalTransform(this);
}

double cSimObj::GetMass() const
{
	return 1 / mBody->getInvMass();
}

tVector cSimObj::WorldToLocalPos(const tVector& world_pos) const
{
	tMatrix world_to_local = GetLocalTransform();
	tVector local_pt = world_pos;

	local_pt[3] = 1;
	local_pt = world_to_local * local_pt;
	local_pt[3] = 0;

	return local_pt;
}

tVector cSimObj::LocalToWorldPos(const tVector& local_pos) const
{
	tMatrix local_to_world = GetWorldTransform();
	tVector world_pos = local_pos;

	world_pos[3] = 1;
	world_pos = local_to_world * world_pos;
	world_pos[3] = 0;

	return world_pos;
}

tMatrix3 cSimObj::GetLocalToWorldRotMat() const
{
	tMatrix local_to_world = GetWorldTransform();
	tMatrix3 mat = local_to_world.block(0, 0, 3, 3);
	return mat;
}

void cSimObj::ApplyForce(const tVector& force)
{
	ApplyForce(force, tVector::Zero());
}

void cSimObj::ApplyForce(const tVector& force, const tVector& local_pos)
{
	mWorld->ApplyForce(force, local_pos, this);
}

void cSimObj::ApplyTorque(const tVector& torque)
{
	mWorld->ApplyTorque(torque, this);
}

void cSimObj::ClearForces()
{
	mBody->clearForces();
}

void cSimObj::RegisterContact()
{
	RegisterContact(cContactManager::gFlagAll, cContactManager::gFlagAll);
}

void cSimObj::RegisterContact(int contact_flags, int filter_flags)
{
	if (!mContactHandle.IsValid())
	{
		mContactHandle = mWorld->RegisterContact(contact_flags, filter_flags);
		assert(mContactHandle.IsValid());
	}
	else
	{
		assert(false); // already registered contact
	}
}

void cSimObj::UpdateContact(int contact_flags, int filter_flags)
{
	mContactHandle.mFlags = contact_flags;
	mContactHandle.mFilterFlags = filter_flags;

	if (mContactHandle.IsValid())
	{
		mWorld->UpdateContact(mContactHandle);
	}
}

const cContactManager::tContactHandle& cSimObj::GetContactHandle() const
{
	return mContactHandle;
}

bool cSimObj::IsInContact() const
{
	bool in_contact = mWorld->IsInContact(mContactHandle);
	return in_contact;
}

tVector cSimObj::GetContactPt() const
{
	return mWorld->GetContactPt(mContactHandle);
}

short cSimObj::GetColGroup() const
{
	return mColGroup;
}

void cSimObj::SetColGroup(short col_group)
{
	mColGroup = col_group;
}

short cSimObj::GetColMask() const
{
	return mColMask;
}

void cSimObj::SetColMask(short col_mask)
{
	mColMask = col_mask;
}

void cSimObj::DisableDeactivation()
{
	mBody->setActivationState(DISABLE_DEACTIVATION);
}

void cSimObj::CalcAABB(tVector& out_min, tVector& out_max) const
{
	mWorld->CalcAABB(this, out_min, out_max);
}

void cSimObj::ConstrainPlane(cWorld::ePlaneCons plane_cons)
{
	mWorld->ConstrainPlane(this, plane_cons);
}

void cSimObj::Constrain(const tVector& linear_factor, const tVector& angular_factor)
{
	mWorld->Constrain(this, linear_factor, angular_factor);
}

cSimObj::eType cSimObj::GetType() const
{
	return mType;
}

cSimObj::eShape cSimObj::GetShape() const
{
	return eShapeInvalid;
}

const std::unique_ptr<btRigidBody>& cSimObj::GetRigidBody() const
{
	return mBody;
}

const std::unique_ptr<btCollisionShape>& cSimObj::GetCollisionShape() const
{
	return mShape;
}

const std::shared_ptr<cWorld>& cSimObj::GetWorld() const
{
	return mWorld;
}

void cSimObj::Init(std::shared_ptr<cWorld> world)
{
	Init(world, cWorld::ePlaneConsNone);
}

void cSimObj::Init(std::shared_ptr<cWorld> world, cWorld::ePlaneCons plane_cons)
{
	RemoveFromWorld();
	mBody->setUserPointer(this);
	AddToWorld(world, plane_cons);
}

void cSimObj::AddToWorld(std::shared_ptr<cWorld> world, cWorld::ePlaneCons plane_cons)
{
	if (mWorld != nullptr)
	{
		RemoveFromWorld();
	}

	mWorld = world;
	mWorld->AddObject(*this);

	ConstrainPlane(plane_cons);
}

void cSimObj::RemoveFromWorld()
{
	if (mWorld != nullptr && mBody != nullptr)
	{
		int num_cons = GetNumConstraints();;
		for (int c = num_cons - 1; c >= 0; --c)
		{
			cWorld::tConstraintHandle cons = GetConstraint(c);
			mWorld->RemoveConstraint(cons);
		}

		if (mCons.IsValid())
		{
			mWorld->RemoveConstraint(mCons);
		}

		mWorld->RemoveObject(*this);
		mWorld.reset();
		mBody.reset();

	}
}

int cSimObj::GetNumConstraints() const
{
	return mBody->getNumConstraintRefs();
}

cWorld::tConstraintHandle cSimObj::GetConstraint(int c) const
{
	cWorld::tConstraintHandle handle;
	handle.mCons = mBody->getConstraintRef(c);
	return handle;
}

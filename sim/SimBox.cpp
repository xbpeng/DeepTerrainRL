#include "SimBox.h"

cSimBox::tParams::tParams()
{
	mType = eTypeDynamic;
	mMass = 1;
	mFriction = 0.9;
	mPos = tVector(0, 0, 0, 0);
	mSize = tVector(1, 1, 1, 0);
	mAxis = tVector(0, 0, 1, 1);
	mVel.setZero();
	mTheta = 0;
}

cSimBox::cSimBox()
{
}

cSimBox::~cSimBox()
{
}

void cSimBox::Init(std::shared_ptr<cWorld> world, const tParams& params)
{
	mType = params.mType;
	btScalar mass = (params.mType == eTypeDynamic) ? static_cast<btScalar>(params.mMass) : 0;

	mShape = world->BuildBoxShape(params.mSize);

	btVector3 inertia(0, 0, 0);
	mShape->calculateLocalInertia(mass, inertia);
	btRigidBody::btRigidBodyConstructionInfo cons_info(mass, this, mShape.get(), inertia);
	mBody = std::unique_ptr<btRigidBody>(new btRigidBody(cons_info));
	mBody->setFriction(static_cast<btScalar>(params.mFriction));
	
	cSimObj::Init(world);
	SetPos(params.mPos);
	SetLinearVelocity(params.mVel);
	SetRotation(params.mAxis, params.mTheta);
}

tVector cSimBox::GetSize() const
{
	return mWorld->GetBoxSize(this);
}

cSimBox::eShape cSimBox::GetShape() const
{
	return eShapeBox;
}
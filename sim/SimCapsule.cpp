#include "SimCapsule.h"

cSimCapsule::tParams::tParams()
{
	mType = eTypeDynamic;
	mMass = 1;
	mFriction = 0.9;
	mPos = tVector(0, 0, 0, 0);
	mHeight = 1;
	mRadius = 1;
	mAxis = tVector(0, 0, 1, 1);
	mTheta = 0;
}

cSimCapsule::cSimCapsule()
{
}

cSimCapsule::~cSimCapsule()
{
}

void cSimCapsule::Init(std::shared_ptr<cWorld> world, const tParams& params)
{
	mType = params.mType;
	btScalar mass = (params.mType == eTypeDynamic) ? static_cast<btScalar>(params.mMass) : 0;
	
	mShape = world->BuildCapsuleShape(params.mRadius, params.mHeight);
	
	btVector3 inertia(0, 0, 0);
	mShape->calculateLocalInertia(mass, inertia);
	btRigidBody::btRigidBodyConstructionInfo cons_info(mass, this, mShape.get(), inertia);
	mBody = std::unique_ptr<btRigidBody>(new btRigidBody(cons_info));
	mBody->setFriction(static_cast<btScalar>(params.mFriction));

	cSimObj::Init(world);
	SetPos(params.mPos);
	SetRotation(params.mAxis, params.mTheta);

}

double cSimCapsule::GetHeight() const
{
	return mWorld->GetCapsuleHeight(this);
}

double cSimCapsule::GetRadius() const
{
	return mWorld->GetCapsuleRadius(this);
}

cSimCapsule::eShape cSimCapsule::GetShape() const
{
	return eShapeCapsule;
}
#include "SimPlane.h"

cSimPlane::tParams::tParams()
{
	mMass = 0;
	mFriction = 0.9;
	mNormal = tVector(0, 1, 0, 0);
	mOrigin = tVector::Zero();
}

cSimPlane::cSimPlane()
{
}

cSimPlane::~cSimPlane()
{
}

void cSimPlane::Init(std::shared_ptr<cWorld> world, const tParams& params)
{
	mType = eTypeStatic;
	mShape = world->BuildPlaneShape(params.mNormal, params.mOrigin);

	btRigidBody::btRigidBodyConstructionInfo cons_info(0, this, mShape.get(), btVector3(0, 0, 0));
	mBody = std::unique_ptr<btRigidBody>(new btRigidBody(cons_info));
	mBody->setFriction(static_cast<btScalar>(params.mFriction));

	cSimObj::Init(world);
}


tVector cSimPlane::GetCoeffs() const
{
	return mWorld->GetPlaneCoeffs(this);
}

cSimPlane::eShape cSimPlane::GetShape() const
{
	return eShapePlane;
}
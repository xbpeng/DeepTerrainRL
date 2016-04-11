#include "GroundFlat.h"

cGroundFlat::tParams::tParams()
{
	mFriction = 0.9;
	mOrigin.setZero();
}

cGroundFlat::cGroundFlat()
{
}

cGroundFlat::~cGroundFlat()
{
}

void cGroundFlat::Init(std::shared_ptr<cWorld> world, const tParams& params)
{
	btVector3 normal = btVector3(0, 1, 0);
	btVector3 origin = btVector3(static_cast<btScalar>(params.mOrigin[0]),
								static_cast<btScalar>(params.mOrigin[1]),
								static_cast<btScalar>(params.mOrigin[2]));
	normal.normalize();
	btScalar w = normal.dot(origin);
	mShape = std::unique_ptr<btCollisionShape>(new btStaticPlaneShape(normal, w));

	btRigidBody::btRigidBodyConstructionInfo cons_info(0, this, mShape.get(), btVector3(0, 0, 0));
	mBody = std::unique_ptr<btRigidBody>(new btRigidBody(cons_info));
	mBody->setFriction(static_cast<btScalar>(params.mFriction));

	cGround::Init(world);
}

double cGroundFlat::SampleHeight(const tVector& pos) const
{
	return cGround::SampleHeight(pos);
}

double cGroundFlat::SampleHeight(const tVector& pos, bool& out_valid_sample) const
{
	const btStaticPlaneShape* shape = reinterpret_cast<btStaticPlaneShape*>(mShape.get());
	btVector3 n = shape->getPlaneNormal();
	btScalar c = shape->getPlaneConstant();

	out_valid_sample = true;
	return n[1] * c;
}

void cGroundFlat::SampleHeight(const Eigen::MatrixXd& pos, Eigen::VectorXd& out_h) const
{
	double h = SampleHeight(tVector::Zero());
	int num_pos = static_cast<int>(pos.rows());
	out_h = Eigen::VectorXd::Ones(num_pos) * h;
}

cGroundFlat::eGroundType cGroundFlat::GetGroundType() const
{
	return eGroundTypeFlat;
}
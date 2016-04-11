#include "Ground.h"

cGround::cGround()
{
	mType = eTypeStatic;
}

cGround::~cGround()
{
}

void cGround::Init(std::shared_ptr<cWorld> world)
{
	cSimObj::Init(world);
	UpdateContact(cWorld::eContactFlagEnvironment, cWorld::eContactFlagAll);
}

void cGround::Update(const tVector& bound_min, const tVector& bound_max)
{
	// *whistle whistle*.... nothing to see here
}

void cGround::Clear()
{
}

double cGround::SampleHeight(const tVector& pos) const
{
	bool dummy_valid = true;
	return SampleHeight(pos, dummy_valid);
}

double cGround::SampleHeight(const tVector& pos, bool& out_valid_sample) const
{
	out_valid_sample = true;
	return 0;
}

void cGround::SampleHeight(const Eigen::MatrixXd& pos, Eigen::VectorXd& out_h) const
{
	int num_pos = static_cast<int>(pos.rows());
	out_h = Eigen::VectorXd::Zero(num_pos);
}

cGround::eGroundType cGround::GetGroundType() const
{
	return eGroundTypeInvalid;
}

void cGround::SetTerrainParams(const Eigen::VectorXd& params)
{
	mTerrainParams = params;
}
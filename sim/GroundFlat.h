#pragma once

#include "Ground.h"

class cGroundFlat : public cGround
{
public:
	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		tParams();

		double mFriction;
		tVector mOrigin;
	};

	cGroundFlat();
	virtual ~cGroundFlat();

	virtual void Init(std::shared_ptr<cWorld> world, const tParams& params);

	virtual double SampleHeight(const tVector& pos) const;
	virtual double SampleHeight(const tVector& pos, bool& out_valid_sample) const;
	virtual void SampleHeight(const Eigen::MatrixXd& pos, Eigen::VectorXd& out_h) const;

	virtual eGroundType GetGroundType() const;

protected:
};
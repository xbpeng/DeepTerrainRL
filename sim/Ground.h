#pragma once

#include "sim/SimObj.h"

class cGround : public cSimObj
{
public:
	enum eGroundType
	{
		eGroundTypeFlat,
		eGroundTypeVar2D,
		eGroundTypeMax,
		eGroundTypeInvalid
	};

	virtual ~cGround();

	virtual void Init(std::shared_ptr<cWorld> world);
	virtual void Update(const tVector& bound_min, const tVector& bound_max);
	virtual void Clear();

	virtual double SampleHeight(const tVector& pos) const;
	virtual double SampleHeight(const tVector& pos, bool& out_valid_sample) const;
	virtual void SampleHeight(const Eigen::MatrixXd& pos, Eigen::VectorXd& out_h) const;

	virtual eGroundType GetGroundType() const;
	virtual void SetTerrainParams(const Eigen::VectorXd& params);

protected:
	Eigen::VectorXd mTerrainParams;

	cGround();
};
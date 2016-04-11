#pragma once

#include "sim/TerrainRLCharController.h"

class cBaseControllerQ : public virtual cTerrainRLCharController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	virtual ~cBaseControllerQ();
	
	virtual int GetPoliActionSize() const;
	virtual void RecordPoliAction(Eigen::VectorXd& out_action) const;

	virtual void BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const;

protected:

	cBaseControllerQ();

	virtual bool ShouldExplore() const;
	virtual void DecideAction(tAction& out_action);
	virtual void ExploitPolicy(tAction& out_action);
	virtual void ExploreAction(tAction& out_action);
};
#include "BaseControllerQ.h"
#include "sim/SimCharacter.h"

cBaseControllerQ::cBaseControllerQ() : cTerrainRLCharController()
{
}

cBaseControllerQ::~cBaseControllerQ()
{
}

int cBaseControllerQ::GetPoliActionSize() const
{
	return GetNumActions();
}

void cBaseControllerQ::RecordPoliAction(Eigen::VectorXd& out_action) const
{
	int action_size = GetPoliActionSize();
	int action_id = GetCurrActionID();
	out_action = Eigen::VectorXd::Zero(action_size);
	out_action[action_id] = 1;
}

void cBaseControllerQ::BuildNNOutputOffsetScale(Eigen::VectorXd& out_offset, Eigen::VectorXd& out_scale) const
{
	int output_size = GetNumActions();
	out_offset = -0.5 * Eigen::VectorXd::Ones(output_size);
	out_scale = 2 * Eigen::VectorXd::Ones(output_size);
}

bool cBaseControllerQ::ShouldExplore() const
{
	bool explore = false;
	if (EnabledExplore())
	{
		double rand = cMathUtil::RandDouble(0, 1);
		double exp_rate = GetExpRate();
		explore = rand < exp_rate;
	}
	return explore;
}

void cBaseControllerQ::DecideAction(tAction& out_action)
{
	bool explore = ShouldExplore();

	if (explore)
	{
		mIsOffPolicy = true;
		ExploreAction(out_action);
	}
	else
	{
		mIsOffPolicy = false;
		ExploitPolicy(out_action);
	}
}

void cBaseControllerQ::ExploitPolicy(tAction& out_action)
{
	Eigen::VectorXd action;
	mNet.Eval(mPoliState, action);
	int a = 0;
	double max_val = action.maxCoeff(&a);

	BuildBaseAction(a, out_action);

#if defined(ENABLE_DEBUG_PRINT)
	printf("action: %i (%.3f) ", a, max_val);
	for (int i = 0; i < action.size(); ++i)
	{
		printf("%.3f ", action[i]);
	}
	printf("\n");
#endif

#if defined(ENABLE_DEBUG_VISUALIZATION)
	mPoliValLog.Add(max_val);
	mVisNNOutput = action;
#endif // ENABLE_DEBUG_VISUALIZATION
}

void cBaseControllerQ::ExploreAction(tAction& out_action)
{
	BuildRandBaseAction(out_action);
}

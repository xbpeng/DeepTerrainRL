#include "DogControllerMACE.h"

cDogControllerMACE::cDogControllerMACE() : cTerrainRLCharController(),
										cDogController(),
										cBaseControllerMACE()
{
	mExpBaseActionRate = 0.2;
	mExpNoise = 0.2;
}

cDogControllerMACE::~cDogControllerMACE()
{
}

void cDogControllerMACE::Reset()
{
	cBaseControllerMACE::Reset();
	cDogController::Reset();
}

bool cDogControllerMACE::IsCurrActionCyclic() const
{
	return false;
}

void cDogControllerMACE::UpdateAction()
{
	cBaseControllerMACE::UpdateAction();
	cDogController::UpdateAction();
}

void cDogControllerMACE::BuildBaseAction(int action_id, tAction& out_action) const
{
	cDogController::BuildBaseAction(action_id, out_action);
	out_action.mID = AssignFragID(action_id);
}

void cDogControllerMACE::ProcessCommand(tAction& out_action)
{
	cBaseControllerMACE::ProcessCommand(out_action);
	cDogController::ProcessCommand(out_action);
}

int cDogControllerMACE::AssignFragID(int a_id) const
{
#if defined(DISABLE_INIT_ACTOR_BIAS)
	return cMathUtil::RandInt(0, GetNumActionFrags());
#else
	int frag_id = 0;
	int num_frags = GetNumActionFrags();
	if (num_frags > 0)
	{
		const tBlendAction& blend = mActions[a_id];
		int id0 = blend.mParamIdx0;
		int id1 = blend.mParamIdx1;

		if (id0 >= num_frags && id1 >= num_frags)
		{
			// randomly assign a fragment
			frag_id = cMathUtil::RandInt(0, num_frags);
		}
		else if (id0 >= num_frags)
		{
			frag_id = id1;
		}
		else if (id1 >= num_frags)
		{
			frag_id = id0;
		}
		else
		{
			frag_id = (cMathUtil::FlipCoin()) ? id0 : id1;

			int num_ctrl_params = static_cast<int>(mCtrlParams.size());
			int num_ctrl_copies = num_frags / num_ctrl_params;
			int num_ctrl_remainder = num_frags % num_ctrl_params;

			int num_copies = num_ctrl_copies;
			if (frag_id < num_ctrl_remainder)
			{
				++num_copies;
			}

			int frag_id_offset = cMathUtil::RandInt(0, num_copies);
			frag_id_offset *= num_ctrl_params;
			frag_id += frag_id_offset;
		}
	}
	return frag_id;
#endif
}

void cDogControllerMACE::BuildActorBias(int a_id, Eigen::VectorXd& out_bias) const
{
	int num_ctrl_params = static_cast<int>(mCtrlParams.size());
	int ctrl_idx = a_id % num_ctrl_params;
	const Eigen::VectorXd& ctrl_params = GetCtrlParams(ctrl_idx);
	GetOptParams(ctrl_params, out_bias);
}
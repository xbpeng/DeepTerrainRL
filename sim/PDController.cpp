#include "PDController.h"
#include <iostream>

#include "sim/SimCharacter.h"
#include "util/FileUtil.h"

const std::string gPDControllersKey = "PDControllers";
const std::string gPDParamKeys[cPDController::eParamMax] =
{
	"JointID",
	"Kp",
	"Kd",
	"TorqueLim",
	"TargetTheta",
	"TargetVel",
	"UseWorldCoord"
};

bool cPDController::LoadParams(const std::string& file, Eigen::MatrixXd& out_buffer)
{
	std::ifstream f_stream(file);
	Json::Reader reader;
	Json::Value root;
	bool succ = reader.parse(f_stream, root);
	f_stream.close();

	if (succ)
	{
		if (!root[gPDControllersKey].isNull())
		{
			const Json::Value& pd_controllers = root[gPDControllersKey];
			int num_ctrls = pd_controllers.size();
			out_buffer.resize(num_ctrls, eParamMax);

			for (int i = 0; i < num_ctrls; ++i)
			{
				tParams curr_params;
				Json::Value json_pd_ctrl = pd_controllers.get(i, 0);
				bool succ_def = ParsePDParams(json_pd_ctrl, curr_params);
				if (succ_def)
				{
					int joint_id = i;
					curr_params[eParamJointID] = i;
					out_buffer.row(i) = curr_params;
				}
				else
				{
					succ = false;
					break;
				}
			}
		}
		
	}
	else
	{
		printf("Failed to load PD controller parameters from %s\n", file.c_str());
	}

	return succ;
}

bool cPDController::ParsePDParams(const Json::Value& root, tParams& out_params)
{
	bool succ = true;

	out_params.setZero();
	for (int i = 0; i < eParamMax; ++i)
	{
		const std::string& curr_key = gPDParamKeys[i];
		if (!root[curr_key].isNull() && root[curr_key].isNumeric())
		{
			Json::Value json_val = root[curr_key];
			double val = json_val.asDouble();
			out_params[i] = val;
		}
	}

	return succ;
}

cPDController::cPDController()
	: cController()
{
	Clear();
}

cPDController::~cPDController()
{
}

void cPDController::Init(cSimCharacter* character, const tParams& params)
{
	cController::Init(character);
	mParams = params;
	mValid = true;

	cJoint& joint = GetJoint();
	double torque_lim = GetTorqueLimit();
	joint.SetTorqueLimit(torque_lim);
}

void cPDController::Clear()
{
	cController::Clear();
	
	mParams[eParamJointID] = static_cast<double>(cKinTree::gInvalidJointID);
	mParams[eParamKp] = 0;
	mParams[eParamKd] = 0;
	mParams[eParamTorqueLim] = std::numeric_limits<double>::infinity();
	mParams[eParamTargetTheta] = 0;
	mParams[eParamTargetVel] = 0;
	mParams[eParamUseWorldCoord] = 0;
}

void cPDController::Update(double time_step)
{
	if (IsActive())
	{
		cJoint& joint = GetJoint();
		tVector torque = CalcTorque();
		joint.AddTorque(torque);
	}
}

cJoint& cPDController::GetJoint()
{
	return mChar->GetJoint(GetJointID());
}

const cJoint& cPDController::GetJoint() const
{
	return mChar->GetJoint(GetJointID());
}

void cPDController::SetKp(double kp)
{
	mParams[eParamKp] = kp;
}

double cPDController::GetKp() const
{
	return mParams[eParamKp];
}

double cPDController::GetTorqueLimit() const
{
	return mParams[eParamTorqueLim];
}

void cPDController::SetKd(double kd)
{
	mParams[eParamKd] = kd;
}

double cPDController::GetKd() const
{
	return mParams[eParamKd];
}

void cPDController::SetTargetTheta(double theta)
{
	mParams[eParamTargetTheta] = theta;
}

void cPDController::SetTargetVel(double vel)
{
	mParams[eParamTargetVel] = vel;
}

void cPDController::SetUseWorldCoord(bool use)
{
	mParams[eParamUseWorldCoord] = (use) ? 1 : 0;
}

bool cPDController::UseWorldCoord() const
{
	return mParams[eParamUseWorldCoord] != 0;
}

double cPDController::CalcTheta() const
{
	const cJoint& joint = GetJoint();
	tVector rot_axis;
	double theta = 0;
	if (UseWorldCoord())
	{
		joint.GetChildRotation(rot_axis, theta);
		tVector axis_world = joint.CalcAxisWorld();
		theta *= rot_axis.dot(axis_world);
	}
	else
	{
		joint.CalcRotation(rot_axis, theta);
	}

	return theta;
}

double cPDController::CalcVel() const
{
	const cJoint& joint = GetJoint();
	const tVector& axis_rel = joint.GetAxisRel();

	tVector joint_vel = joint.CalcJointVelRel();
	double vel = joint_vel.dot(axis_rel);
	return vel;
}

double cPDController::CalcThetaErr() const
{
	double theta = CalcTheta();
	double tar_theta = GetTargetTheta();
	double theta_err = tar_theta - theta;
	return theta_err;
}

double cPDController::CalcVelErr() const
{
	double vel = CalcVel();
	double tar_vel = GetTargetVel();
	double vel_err = tar_vel - vel;
	return vel_err;
}

double cPDController::CalcTargetTheta(double torque) const
{
	double theta = CalcTheta();
	double vel = CalcVel();

	double kp = mParams[eParamKp];
	double kd = mParams[eParamKd];

	double tar_vel = GetTargetVel();
	double tar_theta = theta + 1 / kp * (torque - kd * (tar_vel - vel));
	return tar_theta;
}

double cPDController::CalcTargetVel(double torque) const
{
	double theta = CalcTheta();
	double vel = CalcVel();

	double kp = mParams[eParamKp];
	double kd = mParams[eParamKd];

	double tar_theta = GetTargetTheta();
	double tar_vel = vel + 1 / kd * (torque - kp * (tar_theta - theta));
	return tar_vel;
}

double cPDController::GetTargetTheta() const
{
	return mParams[eParamTargetTheta];
}

double cPDController::GetTargetVel() const
{
	return mParams[eParamTargetVel];
}

bool cPDController::IsActive() const
{
	bool active = cController::IsActive();
	//active &= GetTorqueLimit() > 0;
	return active;
}

tVector cPDController::CalcTorque() const
{
	const cJoint& joint = GetJoint();
	const tVector& axis_rel = joint.GetAxisRel();
	
	double kp = mParams[eParamKp];
	double kd = mParams[eParamKd];

	double theta_err = CalcThetaErr();
	double vel_err = CalcVelErr();
	double t = kp * theta_err + kd * vel_err;
	tVector torque = axis_rel * t;

	return torque;
}

int cPDController::GetJointID() const
{
	return static_cast<int>(mParams[eParamJointID]);
}

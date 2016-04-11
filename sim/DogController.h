#pragma once

#include "sim/PDController.h"
#include "sim/ImpPDController.h"
#include "sim/SimDog.h"
#include "sim/RBDModel.h"
#include "sim/TerrainRLCharController.h"

class cDogController : public virtual cTerrainRLCharController
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eStateParam
	{
		eStateParamSpineCurve,
		eStateParamShoulder,
		eStateParamElbow,
		eStateParamHip,
		eStateParamKnee,
		eStateParamAnkle,
		eStateParamMax
	};
	typedef Eigen::Matrix<double, eStateParamMax, 1> tStateParams;

	enum eMiscParam
	{
		eMiscParamTransTime,
		eMiscParamCv,
		eMiscParamBackForceX,
		eMiscParamBackForceY,
		eMiscParamFrontForceX,
		eMiscParamFrontForceY,
		eMiscParamMax
	};
	typedef Eigen::Matrix<double, eMiscParamMax, 1> tMiscParams;

	enum eState
	{
		eStateBackStance,
		eStateExtend,
		eStateFrontStance,
		eStateGather,
		eStateMax,
		eStateInvalid,
	};
	
	struct tStateDef
	{
		std::string mName;
		bool mTransTime;
		cSimDog::eJoint mTransContact;
		eState mNext;
	};

	struct tParamInfo
	{
		bool mIsOptParam;
		double mMin;
		double mMax;
	};

	cDogController();
	virtual ~cDogController();

	virtual void Init(cSimCharacter* character, const tVector& gravity, const std::string& param_file);
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_step);
	
	virtual void SeCtrlStateParams(eState state, const tStateParams& params);

	virtual void TransitionState(int state);
	virtual void TransitionState(int state, double phase);
	virtual void SetTransTime(double time);
	virtual int GetNumStates() const;

	virtual void SetMode(eMode mode);
	virtual void CommandAction(int action_id);
	virtual void CommandRandAction();
	virtual int GetDefaultAction() const;
	virtual void SetDefaultAction(int action_id);
	virtual int GetNumActions() const;

	virtual void BuildCtrlOptParams(int ctrl_params_id, Eigen::VectorXd& out_params) const;
	virtual void SetCtrlParams(int ctrl_params_id, const Eigen::VectorXd& params);
	virtual void SetCtrlOptParams(int ctrl_params_id, const Eigen::VectorXd& params);
	virtual void BuildActionOptParams(int action_id, Eigen::VectorXd& out_params) const;
	virtual int GetNumParams() const;
	virtual int GetNumOptParams() const;
	virtual void FetchOptParamScale(Eigen::VectorXd& out_scale) const;
	virtual void OutputOptParams(const std::string& file, const Eigen::VectorXd& params) const;
	virtual void OutputOptParams(FILE* f, const Eigen::VectorXd& params) const;
	
	virtual void SetParams(const Eigen::VectorXd& params);
	virtual void BuildOptParams(Eigen::VectorXd& out_params) const;
	virtual void SetOptParams(const Eigen::VectorXd& opt_params);
	virtual void SetOptParams(const Eigen::VectorXd& opt_params, Eigen::VectorXd& out_params) const;

	virtual void ReadParams(const std::string& file);
	virtual void ReadParams(std::ifstream& f_stream);
	
	virtual void BuildFromMotion(int ctrl_params_id, const cMotion& motion);

	virtual double CalcReward() const;
	virtual tVector GetTargetVel() const;

	virtual double GetPrevCycleTime() const;
	virtual const tVector& GetPrevDistTraveled() const;

protected:
	struct tBlendAction
	{
		int mID;
		int mParamIdx0;
		int mParamIdx1;
		double mBlend;
		bool mCyclic;
	};

	tVector mGravity;
	cImpPDController mImpPDCtrl;

	std::shared_ptr<cRBDModel> mRBDModel;
	Eigen::MatrixXd mJacobian;

	std::vector<Eigen::VectorXd> mCtrlParams;
	int mDefaultAction;
	std::vector<tBlendAction> mActions;
	std::stack<int> mCommands;
	bool mEnableGravityCompensation;

	double mPrevCycleTime;
	double mCurrCycleTime;
	tVector mPrevCOM;
	tVector mPrevDistTraveled;
	double mPrevStumbleCount;
	double mCurrStumbleCount;

	virtual void ResetParams();

	virtual bool LoadControllers(const std::string& file);
	virtual bool ParseControllers(const Json::Value& root);
	virtual bool ParseControllerFiles(const Json::Value& root);
	virtual bool ParseActions(const Json::Value& root);
	virtual bool ParseAction(const Json::Value& root, tBlendAction& out_action) const;

	virtual bool HasCtrlParams() const;
	virtual void UpdateState(double time_step);
	virtual void UpdateAction();
	virtual void UpdateRBDModel();
	virtual void UpdatePDCtrls(double time_step, Eigen::VectorXd& out_tau);
	virtual void UpdateStumbleCounter(double time_step);
	virtual void ApplyFeedback();
	virtual void ApplyGravityCompensation(Eigen::VectorXd& out_tau);
	virtual void ApplyVirtualForces(Eigen::VectorXd& out_tau);

	virtual const tStateDef& GetCurrStateDef() const;
	virtual tStateParams GetCurrParams() const;

	virtual void SetStateParams(const tStateParams& params);
	virtual void SetupPassiveMode();
	virtual double GetTransTime() const;
	virtual double GetCv() const;
	virtual tVector GetBackForce() const;
	virtual tVector GetFrontForce() const;

	virtual bool CheckContact(cSimDog::eJoint joint_id) const;
	virtual bool IsActiveVFEffector(cSimDog::eJoint joint_id) const;
	virtual tVector GetEffectorVF(cSimDog::eJoint joint_id) const;
	virtual Eigen::MatrixXd BuildContactBasis(const Eigen::VectorXd& pose, bool& out_has_support) const;

	virtual const std::string& GetStateName(eState state) const;
	virtual const std::string& GetStateParamName(eStateParam param) const;

	virtual void GetOptParams(const Eigen::VectorXd& ctrl_params, Eigen::VectorXd& out_opt_params) const;
	virtual std::string BuildOptParamsJson(const Eigen::VectorXd& opt_params) const;
	virtual std::string BuildParamsJson(const Eigen::VectorXd& params) const;
	virtual void BuildStateParamsFromPose(const Eigen::VectorXd& pose, tStateParams& out_params);
	virtual double CalcSpineCurve(const Eigen::VectorXd& pose) const;

	virtual const Eigen::VectorXd& GetCtrlParams(int ctrl_id) const;

	virtual bool IsCurrActionCyclic() const;
	virtual void ApplyAction(int action_id);
	virtual void ApplyAction(const tAction& action);
	virtual void NewCycleUpdate();
	virtual void BlendCtrlParams(const tBlendAction& action, Eigen::VectorXd& out_params) const;
	virtual void PostProcessParams(Eigen::VectorXd& out_params) const;
	virtual bool IsOptParam(int param_idx) const;
	virtual double GetParamBoundMin(int param_idx) const;
	virtual double GetParamBoundMax(int param_idx) const;

	virtual tVector GetEndEffectorContactPos(int joint_id) const;
	virtual void RecordDistTraveled();

	virtual int PopCommand();
	virtual bool HasCommands() const;
	virtual void ClearCommands();
	virtual void ProcessCommand(tAction& out_action);

	virtual void BuildBaseAction(int action_id, tAction& out_action) const;
};
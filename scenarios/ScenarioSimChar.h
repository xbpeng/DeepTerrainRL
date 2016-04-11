#pragma once

#include "scenarios/Scenario.h"
#include "sim/World.h"
#include "sim/SimBox.h"
#include "sim/SimPlane.h"
#include "sim/SimCapsule.h"
#include "sim/SimCharacter.h"
#include "sim/Perturb.h"
#include "sim/Ground.h"
#include "sim/TerrainGen2D.h"

class cScenarioSimChar : public cScenario
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum eCharCtrl
	{
		eCharCtrlNone,
		eCharCtrlDog,
		eCharCtrlDogCacla,
		eCharCtrlDogMACE,
		eCharCtrlGoatMACE,
		eCharCtrlRaptor,
		eCharCtrlRaptorCacla,
		eCharCtrlRaptorMACE,
		eCharCtrlMax
	};

	enum eCharType
	{
		eCharNone,
		eCharDog,
		eCharRaptor,
		eCharMax
	};

	struct tObjEntry
	{
		tObjEntry();
		std::shared_ptr<cSimObj> mObj;
		double mEndTime;
	};

	cScenarioSimChar();
	virtual ~cScenarioSimChar();

	virtual void ParseArgs(const cArgParser& parser);
	virtual void Init();
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_elapsed);

	virtual const std::shared_ptr<cSimCharacter>& GetCharacter() const;
	virtual const std::shared_ptr<cWorld>& GetWorld() const;
	virtual tVector GetCharPos() const;
	virtual const std::shared_ptr<cGround>& GetGround() const;

	virtual void AddPerturb(const tPerturb& perturb);
	virtual void ApplyRandForce(double min_force, double max_force, 
								double min_dur, double max_dur, cSimObj* obj);
	virtual void ApplyRandForce();
	virtual cSimObj* RayTest(const tVector& beg, const tVector& end, tVector& out_intersection) const;

	virtual void SetTerrainParamsLerp(double lerp);
	virtual int GetNumTerrainParams() const;
	
	virtual void OutputCharState(const std::string& out_file) const;

	virtual bool HasFallen() const;
	virtual bool HasStumbled() const;
	virtual eCharType GetCharType() const;

	virtual void SpawnProjectile();
	virtual void SpawnBigProjectile();
	virtual const std::vector<tObjEntry>& GetObjs() const;

	virtual void SetPreSubstepCallback(tTimeCallbackFunc func);
	virtual void SetPostSubstepCallback(tTimeCallbackFunc func);

	virtual std::string GetName() const;

protected:

	double mTime; // in seconds
	int mNumUpdateSteps;
	int mNumSimSubsteps;
	double mWorldScale;
	std::string mCharacterFile;
	std::string mCharStateFile;

	tVector mGravity;

	std::shared_ptr<cWorld> mWorld;
	std::shared_ptr<cGround> mGround;
	std::shared_ptr<cSimCharacter> mChar;
	eCharType mCharType;
	eCharCtrl mCharCtrl;
	std::string mExpLayer; // mostly for action exploration

	bool mValidCharInitPos;
	tVector mCharInitPos;
	
	cTerrainGen2D::eType mTerrainType;
	std::vector<Eigen::VectorXd> mTerrainParams;
	double mTerrainBlend;

	double mMinPerturb;
	double mMaxPerturb;
	double mMinPerturbDuration;
	double mMaxPerturbDuration;

	std::vector<tObjEntry> mObjs;
	tTimeCallbackFunc mPreSubstepCallback;
	tTimeCallbackFunc mPostSubstepCallback;

	virtual void BuildWorld();
	virtual bool BuildCharacter();
	virtual void BuildGround();
	virtual bool BuildController(std::shared_ptr<cCharController>& out_ctrl);
	virtual bool BuildDogController(std::shared_ptr<cCharController>& out_ctrl) const;
	virtual bool BuildDogControllerCacla(std::shared_ptr<cCharController>& out_ctrl) const;
	virtual bool BuildDogControllerMACE(std::shared_ptr<cCharController>& out_ctrl) const;
	
	virtual bool BuildGoatControllerMACE(std::shared_ptr<cCharController>& out_ctrl) const;
	
	virtual bool BuildRaptorController(std::shared_ptr<cCharController>& out_ctrl) const;
	virtual bool BuildRaptorControllerCacla(std::shared_ptr<cCharController>& out_ctrl) const;
	virtual bool BuildRaptorControllerMACE(std::shared_ptr<cCharController>& out_ctrl) const;

	virtual cWorld::ePlaneCons GetCharPlaneCons() const;

	virtual void CreateCharacter(std::shared_ptr<cSimCharacter>& out_char) const;
	virtual tVector GetDefaultCharPos() const;
	virtual void InitCharacterPos(std::shared_ptr<cSimCharacter>& out_char) const;

	virtual void UpdateWorld(double time_step);
	virtual void UpdateCharacter(double time_step);
	virtual void UpdateGround();
	virtual void ResetGround();

	virtual void PreSubstepUpdate(double time_step);
	virtual void PostSubstepUpdate(double time_step);

	virtual double GetViewDist() const;

	virtual void ParseCharType(const std::string& char_ctrl_str, eCharType& out_char_type) const;
	virtual void ParseCharCtrl(const std::string& char_ctrl_str, eCharCtrl& out_char_ctrl) const;
	virtual void ParseTerrainParams(const cArgParser& parser, cTerrainGen2D::eType& out_type, 
									std::vector<Eigen::VectorXd>& out_params);

	virtual void UpdateObjs(double time_step);
	virtual void ClearObjs();

	virtual void SpawnProjectile(double density, double min_size, double max_size,
									double min_speed, double max_speed, double y_offset, double life_time);
};

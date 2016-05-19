#include "ScenarioSimChar.h"

#include <memory>
#include <ctime>
#include "sim/SimDog.h"
#include "sim/SimRaptor.h"
#include "sim/DogControllerQ.h"
#include "sim/DogControllerCacla.h"
#include "sim/DogControllerMACE.h"
#include "sim/GoatControllerMACE.h"
#include "sim/RaptorControllerQ.h"
#include "sim/RaptorControllerCacla.h"
#include "sim/RaptorControllerMACE.h"
#include "sim/GroundFlat.h"
#include "sim/GroundVar2D.h"

//#define SIM_CHAR_PROFILER

const tVector gLineColor = tVector(0, 0, 0, 1);
const double gCharViewDistPad = 1;
const double gGroundSpawnOffset = -1; // some padding to prevent parts of character from getting spawned inside obstacles

const std::string gCharName[cScenarioSimChar::eCharMax] =
{
	"none",
	"dog",
	"raptor"
};

const std::string gCharCtrlName[cScenarioSimChar::eCharCtrlMax] =
{
	"none",
	"dog",
	"dog_cacla",
	"dog_mace",
	"goat_mace",
	"raptor",
	"raptor_cacla",
	"raptor_mace"
};

cScenarioSimChar::tObjEntry::tObjEntry()
{
	mObj = nullptr;
	mEndTime = INFINITY;
}

cScenarioSimChar::cScenarioSimChar()
{
	mTime = 0;
	mNumUpdateSteps = 20;
	mNumSimSubsteps = 1;
	mWorldScale = 1;
	mCharType = eCharNone;
	mCharCtrl = eCharCtrlNone;
	mExpLayer = "";
	mTerrainType = cTerrainGen2D::eTypeFlat;
	mTerrainBlend = 0;

	mMinPerturb = 50;
	mMaxPerturb = 100;
	mMinPerturbDuration = 0.1;
	mMaxPerturbDuration = 0.5;

	mGravity = gGravity;

	mPreSubstepCallback = nullptr;
	mPostSubstepCallback = nullptr;
}

cScenarioSimChar::~cScenarioSimChar()
{

}

void cScenarioSimChar::ParseArgs(const cArgParser& parser)
{
	bool succ = true;
	succ = parser.ParseString("character_file", mCharacterFile);
	if (!succ)
	{
		printf("No character file specified.\n");
	}

	parser.ParseString("state_file", mCharStateFile);
	parser.ParseDouble("world_scale", mWorldScale);

	parser.ParseDouble("min_perturb", mMinPerturb);
	parser.ParseDouble("max_perturb", mMaxPerturb);
	parser.ParseDouble("min_pertrub_duration", mMinPerturbDuration);
	parser.ParseDouble("max_perturb_duration", mMaxPerturbDuration);
	parser.ParseInt("num_update_steps", mNumUpdateSteps);
	parser.ParseInt("num_sim_substeps", mNumSimSubsteps);
	parser.ParseString("exp_layer", mExpLayer);

	std::string char_type_str = "";
	parser.ParseString("char_type", char_type_str);
	ParseCharType(char_type_str, mCharType);

	std::string char_ctrl_str = "";
	parser.ParseString("char_ctrl", char_ctrl_str);
	ParseCharCtrl(char_ctrl_str, mCharCtrl);

	ParseTerrainParams(parser, mTerrainType, mTerrainParams);
	parser.ParseDouble("terrain_blend", mTerrainBlend);

	mValidCharInitPos = parser.ParseDouble("char_init_pos_x", mCharInitPos[0]);
}

void cScenarioSimChar::Init()
{
	mTime = 0;

	BuildWorld();
	BuildGround();
	BuildCharacter();

	ClearObjs();
}

void cScenarioSimChar::Reset()
{
	cScenario::Reset();

	mTime = 0;
	mChar->Reset();
	mWorld->Reset();

	ResetGround();
	InitCharacterPos(mChar);
	ClearObjs();
}

void cScenarioSimChar::Clear()
{
	mChar->Clear();
	mGround.reset();
	ClearObjs();
}

void cScenarioSimChar::Update(double time_elapsed)
{
#if defined(SIM_CHAR_PROFILER)
	static int time_count = 0;
	static double avg_time = 0;
	std::clock_t total_time_beg = std::clock();
#endif
	if (time_elapsed <= 0)
	{
		return;
	}

	double prev_time = mTime;
	mTime += time_elapsed;

#if defined(ENABLE_TRAINING)
	mChar->ClearEffortBuffer();
#endif

	double update_step = time_elapsed / mNumUpdateSteps;
	int num_update_steps = (time_elapsed == 0) ? 1 : mNumUpdateSteps;
	for (int i = 0; i < num_update_steps; ++i)
	{  
		PreSubstepUpdate(update_step);

		// order matters!
		UpdateWorld(update_step);
		UpdateGround();
		UpdateCharacter(update_step);
		UpdateObjs(update_step);

		PostSubstepUpdate(update_step);
	}

#if defined(SIM_CHAR_PROFILER)
	std::clock_t total_time_end = std::clock();
	double delta_time = static_cast<double>(total_time_end - total_time_beg) / CLOCKS_PER_SEC;
	++time_count;
	avg_time = avg_time * ((time_count - 1.0) / time_count) + delta_time / time_count;
	printf("Sim Char Update Time: %.8f, count: %i\n", avg_time, time_count);
#endif
}

const std::shared_ptr<cSimCharacter>& cScenarioSimChar::GetCharacter()  const
{
	return mChar;
}

const std::shared_ptr<cWorld>& cScenarioSimChar::GetWorld() const
{
	return mWorld;
}

tVector cScenarioSimChar::GetCharPos() const
{
	return GetCharacter()->GetRootPos();
}

const std::shared_ptr<cGround>& cScenarioSimChar::GetGround() const
{
	return mGround;
}

void cScenarioSimChar::AddPerturb(const tPerturb& perturb)
{
	mWorld->AddPerturb(perturb);
}

void cScenarioSimChar::ApplyRandForce(double min_force, double max_force, 
									double min_dur, double max_dur, cSimObj* obj)
{
	assert(obj != nullptr);
	tPerturb perturb = tPerturb::BuildForce();
	perturb.mObj = obj;
	perturb.mLocalPos.setZero();
	perturb.mPerturb[0] = cMathUtil::RandSign() * cMathUtil::RandDouble(0, 1);
	perturb.mPerturb[1] = cMathUtil::RandSign() * cMathUtil::RandDouble(0, 1);
	perturb.mPerturb[2] = cMathUtil::RandSign() * cMathUtil::RandDouble(0, 1);
	perturb.mPerturb = cMathUtil::RandDouble(min_force, max_force) * perturb.mPerturb.normalized();
	perturb.mDuration = cMathUtil::RandDouble(min_dur, max_dur);

	AddPerturb(perturb);
}

void cScenarioSimChar::ApplyRandForce()
{
	int num_parts = mChar->GetNumBodyParts();
	int part_idx = cMathUtil::RandInt(0, num_parts);
	while (!mChar->IsValidBodyPart(part_idx))
	{
		part_idx = cMathUtil::RandInt(0, num_parts);
	}
	const auto& part = mChar->GetBodyPart(part_idx);
	ApplyRandForce(mMinPerturb, mMaxPerturb, mMinPerturbDuration, mMaxPerturbDuration, part.get());
}

cSimObj* cScenarioSimChar::RayTest(const tVector& beg, const tVector& end, tVector& out_intersection) const
{
	cWorld::tRayTestResults results;
	mWorld->RayTest(beg, end, results);

	if (results.size() > 0)
	{
		cWorld::tRayTestResult& result = results[0];
		if (result.mObj->GetType() != cSimObj::eTypeStatic)
		{
			out_intersection = result.mHitPos;
			return result.mObj;
		}
	}

	return nullptr;
}

void cScenarioSimChar::SetTerrainParamsLerp(double lerp)
{
	int num_params = GetNumTerrainParams();
	if (num_params > 0)
	{
		lerp = cMathUtil::Clamp(lerp, 0.0, num_params - 1.0);

		int idx0 = static_cast<int>(lerp);
		int idx1 = std::min(idx0 + 1, num_params - 1);
		lerp -= idx0;

		const cTerrainGen2D::tParams& params0 = mTerrainParams[idx0];
		const cTerrainGen2D::tParams& params1 = mTerrainParams[idx1];
		cTerrainGen2D::tParams lerp_params = (1 - lerp) * params0 + lerp * params1;

		mGround->SetTerrainParams(lerp_params);
	}
}

int cScenarioSimChar::GetNumTerrainParams() const
{
	return static_cast<int>(mTerrainParams.size());
}

void cScenarioSimChar::OutputCharState(const std::string& out_file) const
{
	tVector root_pos = mChar->GetRootPos();
	double ground_h = mGround->SampleHeight(root_pos);
	tVector offset = root_pos;
	offset[1] = ground_h;
	mChar->WriteState(out_file, -offset);
}

bool cScenarioSimChar::HasFallen() const
{
	return mChar->HasFallen();
}

bool cScenarioSimChar::HasStumbled() const
{
	return mChar->HasStumbled();
}

cScenarioSimChar::eCharType cScenarioSimChar::GetCharType() const
{
	return mCharType;
}

std::string cScenarioSimChar::GetName() const
{
	return "Sim Character";
}

bool cScenarioSimChar::BuildCharacter()
{
	CreateCharacter(mChar);

	cSimCharacter::tParams char_params;
	char_params.mPos = GetDefaultCharPos();
	char_params.mCharFile = mCharacterFile;
	char_params.mStateFile = mCharStateFile;
	char_params.mPlaneCons = GetCharPlaneCons();

	bool succ = mChar->Init(mWorld, char_params);
	if (succ)
	{
		mChar->RegisterContacts(cWorld::eContactFlagCharacter, cWorld::eContactFlagEnvironment);
		InitCharacterPos(mChar);

		std::shared_ptr<cCharController> ctrl;
		succ = BuildController(ctrl);
		if (succ && ctrl != nullptr)
		{
			mChar->SetController(ctrl);
		}
	}
	return succ;
}

void cScenarioSimChar::BuildWorld()
{
	cWorld::tParams world_params;
	world_params.mNumSubsteps = mNumSimSubsteps;
	world_params.mGravity = mGravity;
	world_params.mScale = mWorldScale;
	mWorld = std::shared_ptr<cWorld>(new cWorld());
	mWorld->Init(world_params);
}

void cScenarioSimChar::BuildGround()
{
	std::shared_ptr<cGroundVar2D> ground_var2d = std::shared_ptr<cGroundVar2D>(new cGroundVar2D());

	auto terrain_func = cTerrainGen2D::GetTerrainFunc(mTerrainType);
	
	cGroundVar2D::tParams params;
	double char_view_dist = 10;
	params.mSegmentWidth = 2 * char_view_dist;

#if defined(ENABLE_DEBUG_VISUALIZATION)
	params.mSegmentWidth += 50; // hack
#endif

	tVector bound_min = tVector(-char_view_dist + gGroundSpawnOffset, 0, 0, 0);
	tVector bound_max = tVector(char_view_dist + gGroundSpawnOffset, 0, 0, 0);

	mGround = ground_var2d;
	ground_var2d->SetTerrainFunc(terrain_func);

	if (mTerrainParams.size() > 0)
	{
		SetTerrainParamsLerp(mTerrainBlend);
	}
	
	ground_var2d->Init(mWorld, params, bound_min, bound_max);
	
	//std::shared_ptr<cGroundFlat> ground_flat = std::shared_ptr<cGroundFlat>(new cGroundFlat());
	//cGroundFlat::tParams params;
	//ground_flat->Init(mWorld, params);
	//mGround = ground_flat;
}

bool cScenarioSimChar::BuildController(std::shared_ptr<cCharController>& out_ctrl)
{
	bool succ = true;

	switch (mCharCtrl)
	{
	case eCharCtrlNone:
		break;
	case eCharCtrlDog:
		assert(mCharType == eCharDog);
		succ = BuildDogController(out_ctrl);
		break;
	case eCharCtrlDogCacla:
		assert(mCharType == eCharDog);
		succ = BuildDogControllerCacla(out_ctrl);
		break;
	case eCharCtrlDogMACE:
		assert(mCharType == eCharDog);
		succ = BuildDogControllerMACE(out_ctrl);
		break;
	case eCharCtrlGoatMACE:
		assert(mCharType == eCharDog);
		succ = BuildGoatControllerMACE(out_ctrl);
		break;
	case eCharCtrlRaptor:
		assert(mCharType == eCharRaptor);
		succ = BuildRaptorController(out_ctrl);
		break;
	case eCharCtrlRaptorCacla:
		assert(mCharType == eCharRaptor);
		succ = BuildRaptorControllerCacla(out_ctrl);
		break;
	case eCharCtrlRaptorMACE:
		assert(mCharType == eCharRaptor);
		succ = BuildRaptorControllerMACE(out_ctrl);
		break;
	default:
		assert(false && "Failed Building Unsupported Controller"); // unsupported controller
		break;
	}
	
	return succ;
}

bool cScenarioSimChar::BuildDogController(std::shared_ptr<cCharController>& out_ctrl) const
{
	bool succ = true;
	std::shared_ptr<cDogControllerQ> dog_ctrl = std::shared_ptr<cDogControllerQ>(new cDogControllerQ());
	dog_ctrl->SetGround(mGround);
	dog_ctrl->Init(mChar.get(), mGravity, mCharacterFile);

	out_ctrl = dog_ctrl;
	return succ;
}

bool cScenarioSimChar::BuildDogControllerCacla(std::shared_ptr<cCharController>& out_ctrl) const
{
	bool succ = true;
	std::shared_ptr<cDogControllerCacla> dog_ctrl = std::shared_ptr<cDogControllerCacla>(new cDogControllerCacla());
	dog_ctrl->SetGround(mGround);
	dog_ctrl->Init(mChar.get(), mGravity, mCharacterFile);

	out_ctrl = dog_ctrl;
	return succ;
}

bool cScenarioSimChar::BuildDogControllerMACE(std::shared_ptr<cCharController>& out_ctrl) const
{
	bool succ = true;
	std::shared_ptr<cDogControllerMACE> dog_ctrl = std::shared_ptr<cDogControllerMACE>(new cDogControllerMACE());
	dog_ctrl->SetGround(mGround);
	dog_ctrl->Init(mChar.get(), mGravity, mCharacterFile);

	dog_ctrl->SetExpLayer(mExpLayer);

	out_ctrl = dog_ctrl;
	return succ;
}

bool cScenarioSimChar::BuildGoatControllerMACE(std::shared_ptr<cCharController>& out_ctrl) const
{
	bool succ = true;
	std::shared_ptr<cGoatControllerMACE> goat_ctrl = std::shared_ptr<cGoatControllerMACE>(new cGoatControllerMACE());
	goat_ctrl->SetGround(mGround);
	goat_ctrl->Init(mChar.get(), mGravity, mCharacterFile);

	goat_ctrl->SetExpLayer(mExpLayer);

	out_ctrl = goat_ctrl;
	return succ;
}

bool cScenarioSimChar::BuildRaptorController(std::shared_ptr<cCharController>& out_ctrl) const
{
	bool succ = true;
	std::shared_ptr<cRaptorControllerQ> raptor_ctrl = std::shared_ptr<cRaptorControllerQ>(new cRaptorControllerQ());
	raptor_ctrl->SetGround(mGround);
	raptor_ctrl->Init(mChar.get(), mGravity, mCharacterFile);

	out_ctrl = raptor_ctrl;
	return succ;
}

bool cScenarioSimChar::BuildRaptorControllerCacla(std::shared_ptr<cCharController>& out_ctrl) const
{
	bool succ = true;
	std::shared_ptr<cRaptorControllerCacla> raptor_ctrl = std::shared_ptr<cRaptorControllerCacla>(new cRaptorControllerCacla());
	raptor_ctrl->SetGround(mGround);
	raptor_ctrl->Init(mChar.get(), mGravity, mCharacterFile);

	out_ctrl = raptor_ctrl;
	return succ;
}

bool cScenarioSimChar::BuildRaptorControllerMACE(std::shared_ptr<cCharController>& out_ctrl) const
{
	bool succ = true;
	std::shared_ptr<cRaptorControllerMACE> raptor_ctrl = std::shared_ptr<cRaptorControllerMACE>(new cRaptorControllerMACE());
	raptor_ctrl->SetGround(mGround);
	raptor_ctrl->Init(mChar.get(), mGravity, mCharacterFile);

	raptor_ctrl->SetExpLayer(mExpLayer);

	out_ctrl = raptor_ctrl;
	return succ;
}


cWorld::ePlaneCons cScenarioSimChar::GetCharPlaneCons() const
{
	return cWorld::ePlaneConsXY;
}

void cScenarioSimChar::CreateCharacter(std::shared_ptr<cSimCharacter>& out_char) const
{
	if (mCharType == eCharDog)
	{
		out_char = std::shared_ptr<cSimDog>(new cSimDog());
	}
	else if(mCharType == eCharRaptor)
	{
		out_char = std::shared_ptr<cSimRaptor>(new cSimRaptor());
	}
	else
	{
		printf("No valid character specified\n");
		assert(false);
	}
}

tVector cScenarioSimChar::GetDefaultCharPos() const
{
	Eigen::Vector4d out = tVector::Zero();
	
	if (mValidCharInitPos)
	{
		out[0] = mCharInitPos[0];
	}

	return out;
}

void cScenarioSimChar::InitCharacterPos(std::shared_ptr<cSimCharacter>& out_char) const
{
	tVector root_pos = out_char->GetRootPos();

	if (mValidCharInitPos)
	{
		root_pos[0] = mCharInitPos[0];
	}

	double ground_h = mGround->SampleHeight(root_pos);
	root_pos[1] += ground_h;

	out_char->SetRootPos(root_pos);
}

void cScenarioSimChar::UpdateWorld(double time_step)
{
	mWorld->Update(time_step);
}

void cScenarioSimChar::UpdateCharacter(double time_step)
{
	mChar->Update(time_step);
}

void cScenarioSimChar::UpdateGround()
{
	double view_dist = GetViewDist();
	double pad = gCharViewDistPad;
	const tVector bound_min = tVector(-2, 0, 0, 0);
	const tVector bound_max = tVector(view_dist + pad, 0, 0, 0);
	tVector char_pos = mChar->GetRootPos();
	mGround->Update(char_pos + bound_min, char_pos + bound_max);
}

void cScenarioSimChar::ResetGround()
{
	mGround->Clear();

	double char_view_dist = GetViewDist();
	tVector bound_min = tVector(-char_view_dist + gGroundSpawnOffset, 0, 0, 0);
	tVector bound_max = tVector(char_view_dist + gGroundSpawnOffset, 0, 0, 0);

	mGround->Update(bound_min, bound_max);
}

void cScenarioSimChar::PreSubstepUpdate(double time_step)
{
	if (mPreSubstepCallback != nullptr)
	{
		mPreSubstepCallback(time_step);
	}
}

void cScenarioSimChar::PostSubstepUpdate(double time_step)
{
	if (mPostSubstepCallback != nullptr)
	{
		mPostSubstepCallback(time_step);
	}
}

double cScenarioSimChar::GetViewDist() const
{
	const std::shared_ptr<cSimCharacter>& character = GetCharacter();
	const std::shared_ptr<cCharController>& ctrl = character->GetController();
	
	double view_dist = 0;
	if (ctrl != nullptr)
	{
		view_dist = ctrl->GetViewDist();
	}
	return view_dist;
}

void cScenarioSimChar::ParseCharType(const std::string& char_type_str, eCharType& out_char_type) const
{
	bool found = false;
	if (char_type_str == "")
	{
		out_char_type = eCharNone;
		found = true;
	}
	else
	{
		for (int i = 0; i < eCharMax; ++i)
		{
			const std::string& name = gCharName[i];
			if (char_type_str == name)
			{
				out_char_type = static_cast<eCharType>(i);
				found = true;
				break;
			}
		}
	}

	if (!found)
	{
		assert(false && "Unsupported character controller"); // unsupported character controller
	}
}

void cScenarioSimChar::ParseCharCtrl(const std::string& char_ctrl_str, eCharCtrl& out_char_ctrl) const
{
	bool found = false;
	if (char_ctrl_str == "")
	{
		out_char_ctrl = eCharCtrlNone;
		found = true;
	}
	else
	{
		for (int i = 0; i < eCharCtrlMax; ++i)
		{
			const std::string& name = gCharCtrlName[i];
			if (char_ctrl_str == name)
			{
				out_char_ctrl = static_cast<eCharCtrl>(i);
				found = true;
				break;
			}
		}
	}

	if (!found)
	{
		assert(false && "Unsupported character controller"); // unsupported character controller
	}
}

void cScenarioSimChar::ParseTerrainParams(const cArgParser& parser, cTerrainGen2D::eType& out_type, 
											std::vector<Eigen::VectorXd>& out_params)
{
	std::string terrain_file = "";
	parser.ParseString("terrain_file", terrain_file);
	if (terrain_file != "")
	{
		std::ifstream f_stream(terrain_file);
		Json::Reader reader;
		Json::Value root;
		bool succ = reader.parse(f_stream, root);
		f_stream.close();

		if (succ)
		{
			if (!root[cTerrainGen2D::gTypeKey].isNull())
			{
				std::string type_str = root[cTerrainGen2D::gTypeKey].asString();
				cTerrainGen2D::ParseType(type_str, out_type);
			}

			if (!root[cTerrainGen2D::gParamsKey].isNull())
			{
				Json::Value params_arr = root[cTerrainGen2D::gParamsKey];
				assert(params_arr.isArray());
				int num = params_arr.size();

				out_params.resize(num);
				for (int i = 0; i < num; ++i)
				{
					Eigen::VectorXd& curr_params = out_params[i];
					cTerrainGen2D::LoadParams(params_arr.get(i, 0), curr_params);
				}
			}
		}
	}
}


void cScenarioSimChar::UpdateObjs(double time_step)
{
	int idx = 0;
	int num_objs = static_cast<int>(mObjs.size());
	for (size_t i = 0; i < num_objs; ++i)
	{
		const tObjEntry& obj = mObjs[i];
		if (obj.mEndTime > mTime)
		{
			mObjs[idx] = obj;
			++idx;
		}
	}

	if (idx != num_objs)
	{
		mObjs.resize(idx);
	}
}

void cScenarioSimChar::ClearObjs()
{
	mObjs.clear();
}

void cScenarioSimChar::SpawnProjectile()
{
	double density = 100;
	double min_size = 0.1;
	double max_size = 0.3;
	double min_speed = 10;
	double max_speed = 20;
	double life_time = 2;
	double y_offset = 0;
	SpawnProjectile(density, min_size, max_size, min_speed, max_speed, y_offset, life_time);
}

void cScenarioSimChar::SpawnBigProjectile()
{
	double density = 100;
	double min_size = 1.25;
	double max_size = 1.75;
	double min_speed = 11;
	double max_speed = 12;
	double life_time = 2;
	double y_offset = 0.5;
	SpawnProjectile(density, min_size, max_size, min_speed, max_speed, y_offset, life_time);
}

const std::vector<cScenarioSimChar::tObjEntry>& cScenarioSimChar::GetObjs() const
{
	return mObjs;
}

void cScenarioSimChar::SetPreSubstepCallback(tTimeCallbackFunc func)
{
	mPreSubstepCallback = func;
}

void cScenarioSimChar::SetPostSubstepCallback(tTimeCallbackFunc func)
{
	mPostSubstepCallback = func;
}

void cScenarioSimChar::SpawnProjectile(double density, double min_size, double max_size,
	double min_speed, double max_speed, double y_offset,
	double life_time)
{
	double min_dist_x = 1;
	double max_dist_x = 2;
	tVector aabb_min;
	tVector aabb_max;
	mChar->CalcAABB(aabb_min, aabb_max);

	tVector aabb_center = (aabb_min + aabb_max) * 0.5;
	tVector obj_size = tVector(1, 1, 1, 0) * cMathUtil::RandDouble(min_size, max_size);

	double rand_x = (aabb_max[0] - aabb_min[0]) * 0.5 + cMathUtil::RandDouble(min_dist_x, max_dist_x);
	rand_x *= cMathUtil::RandSign();
	rand_x += aabb_center[0];
	double rand_y = cMathUtil::RandDouble(aabb_min[1], aabb_max[1]) + obj_size[1] * 0.5;
	rand_y += y_offset;

	tVector pos = tVector(rand_x, rand_y, aabb_center[2], 0);
	tVector target = tVector(cMathUtil::RandDouble(aabb_min[0], aabb_max[0]),
		cMathUtil::RandDouble(aabb_min[1], aabb_max[1]), aabb_center[2], 0);

	tVector vel = (target - pos).normalized();
	vel *= cMathUtil::RandDouble(min_speed, max_speed);

	cSimBox::tParams params;
	params.mSize = obj_size;
	params.mPos = pos;
	params.mVel = vel;
	params.mFriction = 0.7;
	params.mMass = density * params.mSize[0] * params.mSize[1] * params.mSize[2];
	std::shared_ptr<cSimBox> box = std::shared_ptr<cSimBox>(new cSimBox());
	box->Init(mWorld, params);
	box->ConstrainPlane(GetCharPlaneCons());
	box->UpdateContact(cWorld::eContactFlagObject, cContactManager::gFlagNone);

	tObjEntry obj_entry;
	obj_entry.mObj = box;
	obj_entry.mEndTime = mTime + life_time;
	
	mObjs.push_back(obj_entry);
}
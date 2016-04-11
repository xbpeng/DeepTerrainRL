#include "TerrainGen2D.h"
#include <algorithm>

const float cTerrainGen2D::gVertSpacing = 0.1f;
const std::string cTerrainGen2D::gTypeKey = "Type";
const std::string cTerrainGen2D::gParamsKey = "Params";

const cTerrainGen2D::tParamDef cTerrainGen2D::gParamDefs[] =
{
	{"GapSpacingMin", 4},
	{"GapSpacingMax", 7},
	{"GapWMin", 0.5},
	{"GapWMax", 2},
	{"GapHMin", -2},
	{"GapHMax", -2},

	{"WallSpacingMin", 6},
	{"WallSpacingMax", 8},
	{"WallWMin", 0.2},
	{"WallWMax", 0.2},
	{"WallHMin", 0.25},
	{"WallHMax", 0.5},

	{"StepSpacingMin", 5},
	{"StepSpacingMax", 7},
	{"StepH0Min", 0.1},
	{"StepH0Max", 0.4},
	{"StepH1Min", -0.4},
	{"StepH1Max", -0.1},

	{"BumpHMin", 0},
	{"BumpHMax", 0.03},

	{"NarrowGapSpacingMin", 3},
	{"NarrowGapSpacingMax", 6},
	{"NarrowGapDistMin", 0.1},
	{"NarrowGapDistMax", 0.4},
	{"NarrowGapWMin", 0.15},
	{"NarrowGapWMax", 0.5},
	{"NarrowGapDepthMin", -2},
	{"NarrowGapDepthMax", -2},
	{"NarrowGapCountMin", 1},
	{"NarrowGapCountMax", 4},

	{ "CliffSpacingMin", 5 },
	{ "CliffSpacingMax", 7 },
	{ "CliffH0Min", 0.1 },
	{ "CliffH0Max", 0.4 },
	{ "CliffH1Min", -0.4 },
	{ "CliffH1Max", -0.1 },
	{ "CliffMiniCountMax", 0},

	{"SlopeDeltaRange", 0.25},
	{ "SlopeDeltaMin", -0.35 },
	{"SlopeDeltaMax", 0.35}
};

cTerrainGen2D::tParams cTerrainGen2D::GetDefaultParams()
{
	tParams params;
	assert(sizeof(gParamDefs) / sizeof(gParamDefs[0]) == eParamsMax);
	for (int i = 0; i < eParamsMax; ++i)
	{
		params[i] = gParamDefs[i].mDefaultVal;
	}
	return params;
}

void cTerrainGen2D::LoadParams(const Json::Value& root, Eigen::VectorXd& out_params)
{
	out_params = cTerrainGen2D::GetDefaultParams();
	for (int i = 0; i < eParamsMax; ++i)
	{
		const std::string& name = gParamDefs[i].mName;
		if (!root[name].isNull())
		{
			double val = root[name].asDouble();
			out_params[i] = val;
		}
	}
}

void cTerrainGen2D::ParseType(const std::string& str, eType& out_type)
{
	if (str == "flat"
		|| str == "")
	{
		out_type = eTypeFlat;
	}
	else if (str == "gaps")
	{
		out_type = eTypeGaps;
	}
	else if (str == "steps")
	{
		out_type = eTypeSteps;
	}
	else if (str == "walls")
	{
		out_type = eTypeWalls;
	}
	else if (str == "bumps")
	{
		out_type = eTypeBumps;
	}
	else if (str == "mixed")
	{
		out_type = eTypeMixed;
	}
	else if (str == "narrow_gaps")
	{
		out_type = eTypeNarrowGaps;
	}
	else if (str == "slopes")
	{
		out_type = eTypeSlopes;
	}
	else if (str == "slopes_gaps")
	{
		out_type = eTypeSlopesGaps;
	}
	else if (str == "slopes_steps")
	{
		out_type = eTypeSlopesSteps;
	}
	else if (str == "slopes_walls")
	{
		out_type = eTypeSlopesWalls;
	}
	else if (str == "slopes_mixed")
	{
		out_type = eTypeSlopesMixed;
	}
	else if (str == "slopes_narrow_gaps")
	{
		out_type = eTypeSlopesNarrowGaps;
	}
	else if (str == "cliffs")
	{
		out_type = eTypeCliffs;
	}
	else
	{
		assert(false); // unsupported terrain type
	}
}

cTerrainGen2D::tTerrainFunc cTerrainGen2D::GetTerrainFunc(eType terrain_type)
{
	switch(terrain_type)
	{
	case eTypeGaps:
		return BuildGaps;
	case eTypeSteps:
		return BuildSteps;
	case eTypeWalls:
		return BuildWalls;
	case eTypeBumps:
		return BuildBumps;
	case eTypeMixed:
		return BuildMixed;
	case eTypeNarrowGaps:
		return BuildNarrowGaps;
	case eTypeSlopes:
		return BuildSlopes;
	case eTypeSlopesGaps:
		return BuildSlopesGaps;
	case eTypeSlopesSteps:
		return BuildSlopesSteps;
	case eTypeSlopesWalls:
		return BuildSlopesWalls;
	case eTypeSlopesMixed:
		return BuildSlopesMixed;
	case eTypeSlopesNarrowGaps:
		return BuildSlopesNarrowGaps;
	case eTypeCliffs:
		return BuildCliffs;
	default:
		return BuildFlat;
	}
}



double cTerrainGen2D::BuildFlat(double width, const tParams& params, cRand& rand, std::vector<float>& out_data)
{
	return AddFlat(width, out_data);
}

double cTerrainGen2D::BuildGaps(double width, const tParams& params, cRand& rand, std::vector<float>& out_data)
{
	double spacing_min = params[eParamsGapSpacingMin];
	double spacing_max = params[eParamsGapSpacingMax];
	double gap_w_min = params[eParamsGapWidthMin];
	double gap_w_max = params[eParamsGapWidthMax];
	double gap_d_min = params[eParamsGapDepthMin];
	double gap_d_max = params[eParamsGapDepthMax];

	double total_w = 0;
	while (total_w < width)
	{
		double spacing = rand.RandDouble(spacing_min, spacing_max);
		double w = rand.RandDouble(gap_w_min, gap_w_max);
		double d = rand.RandDouble(gap_d_min, gap_d_max);

		double curr_w = AddBox(spacing, w, d, out_data);
		total_w += curr_w;
	}

	return total_w;
}

double cTerrainGen2D::BuildSteps(double width, const tParams& params, cRand& rand, std::vector<float>& out_data)
{
	double spacing_min = params[eParamsStepSpacingMin];
	double spacing_max = params[eParamsStepSpacingMax];
	double step_h0_min = params[eParamsStepHeight0Min];
	double step_h0_max = params[eParamsStepHeight0Max];
	double step_h1_min = params[eParamsStepHeight1Min];
	double step_h1_max = params[eParamsStepHeight1Max];

	double total_w = 0;
	while (total_w < width)
	{
		double min_h = 0;
		double max_h = 0;

		bool valid_h0 = (step_h0_min != 0 || step_h0_max != 0);
		bool valid_h1 = (step_h1_min != 0 || step_h1_max != 0);
		if (valid_h0 && valid_h1)
		{
			bool heads = rand.FlipCoin();
			min_h = (heads) ? step_h0_min : step_h1_min;
			max_h = (heads) ? step_h0_max : step_h1_max;
		}
		else if (valid_h0)
		{
			min_h = step_h0_min;
			max_h = step_h0_max;
		}
		else
		{
			min_h = step_h1_min;
			max_h = step_h1_max;
		}

		double w = rand.RandDouble(spacing_min, spacing_max);
		double h = rand.RandDouble(min_h, max_h);

		double curr_w = AddStep(w, h, out_data);
		total_w += curr_w;
	}

	return total_w;
}

double cTerrainGen2D::BuildWalls(double width, const tParams& params, cRand& rand, std::vector<float>& out_data)
{
	double spacing_min = params[eParamsWallSpacingMin];
	double spacing_max = params[eParamsWallSpacingMax];
	double wall_w_min = params[eParamsWallWidthMin];
	double wall_w_max = params[eParamsWallWidthMax];
	double wall_g_min = params[eParamsWallHeightMin];
	double wall_g_max = params[eParamsWallHeightMax];

	double total_w = 0;
	while (total_w < width)
	{
		double spacing = rand.RandDouble(spacing_min, spacing_max);
		double w = rand.RandDouble(wall_w_min, wall_w_max);
		double h = rand.RandDouble(wall_g_min, wall_g_max);

		double curr_w = AddBox(spacing, w, h, out_data);
		total_w += curr_w;
	}

	return total_w;
}

double cTerrainGen2D::BuildBumps(double width, const tParams& params, cRand& rand, std::vector<float>& out_data)
{
	double bump_h_min = params[eParamsBumpHeightMin];
	double bump_h_max = params[eParamsBumpHeightMax];

	int beg_idx = static_cast<int>(out_data.size());
	double total_w = BuildFlat(width, params, rand, out_data);
	int end_idx = static_cast<int>(out_data.size());

	OverlayBumps(bump_h_min, bump_h_max, beg_idx, end_idx, rand, out_data);

	return total_w;
}

double cTerrainGen2D::BuildMixed(double width, const tParams& params, cRand& rand, std::vector<float>& out_data)
{
	double total_w = 0;
	const int num_types = 3;
	const double dummy_w = gVertSpacing;

	while (total_w < width)
	{
		double curr_w = 0;
		int rand_type = rand.RandInt(0, num_types);

		if (rand_type == 0)
		{
			curr_w = BuildGaps(dummy_w, params, rand, out_data);
		}
		else if (rand_type == 1)
		{
			curr_w = BuildSteps(dummy_w, params, rand, out_data);
		}
		else if (rand_type == 2)
		{
			curr_w = BuildWalls(dummy_w, params, rand, out_data);
		}

		total_w += curr_w;
	}

	return total_w;
}

double cTerrainGen2D::BuildNarrowGaps(double width, const tParams& params, cRand& rand, std::vector<float>& out_data)
{
	double spacing_min = params[eParamsNarrowGapSpacingMin];
	double spacing_max = params[eParamsNarrowGapSpacingMax];
	double dist_min = params[eParamsNarrowGapDistMin];
	double dist_max = params[eParamsNarrowGapDistMax];
	double gap_w_min = params[eParamsNarrowGapWidthMin];
	double gap_w_max = params[eParamsNarrowGapWidthMax];
	double gap_d_min = params[eParamsNarrowGapDepthMin];
	double gap_d_max = params[eParamsNarrowGapDepthMax];
	int count_min = std::max(1, static_cast<int>(params[eParamsNarrowGapCountMin]));
	int count_max = std::max(1, static_cast<int>(params[eParamsNarrowGapCountMax]));

	double total_w = 0;
	while (total_w < width)
	{
		double spacing = rand.RandDouble(spacing_min, spacing_max);
		int count = rand.RandInt(count_min, count_max + 1);
		for (int i = 0; i < count; ++i)
		{
			double w = rand.RandDouble(gap_w_min, gap_w_max);
			double d = rand.RandDouble(gap_d_min, gap_d_max);

			double curr_w = AddBox(spacing, w, d, out_data);
			total_w += curr_w;

			spacing = rand.RandDouble(dist_min, dist_max);
		}
	}
	
	return total_w;
}

double cTerrainGen2D::BuildSlopes(double width, const tParams& params, cRand& rand, std::vector<float>& out_data)
{
	const double delta_range = std::abs(params[eParamsSlopeDeltaRange]);
	const double delta_min = params[eParamsSlopeDeltaMin];
	const double delta_max = params[eParamsSlopeDeltaMax];

	int beg_idx = static_cast<int>(out_data.size());
	double total_w = BuildFlat(width, params, rand, out_data);
	int end_idx = static_cast<int>(out_data.size());

	OverlaySlopes(delta_range, delta_min, delta_max, 0, beg_idx, end_idx, rand, out_data);

	return total_w;
}

double cTerrainGen2D::BuildSlopesGaps(double width, const tParams& params, cRand& rand, std::vector<float>& out_data)
{
	const double delta_range = std::abs(params[eParamsSlopeDeltaRange]);
	const double delta_min = params[eParamsSlopeDeltaMin];
	const double delta_max = params[eParamsSlopeDeltaMax];

	int beg_idx = static_cast<int>(out_data.size());
	double total_w = BuildGaps(width, params, rand, out_data);
	int end_idx = static_cast<int>(out_data.size());

	OverlaySlopes(delta_range, delta_min, delta_max, 0, beg_idx, end_idx, rand, out_data);

	return total_w;
}

double cTerrainGen2D::BuildSlopesSteps(double width, const tParams& params, cRand& rand, std::vector<float>& out_data)
{
	const double delta_range = std::abs(params[eParamsSlopeDeltaRange]);
	const double delta_min = params[eParamsSlopeDeltaMin];
	const double delta_max = params[eParamsSlopeDeltaMax];

	int beg_idx = static_cast<int>(out_data.size());
	double total_w = BuildSteps(width, params, rand, out_data);
	int end_idx = static_cast<int>(out_data.size());

	OverlaySlopes(delta_range, delta_min, delta_max, 0, beg_idx, end_idx, rand, out_data);

	return total_w;
}

double cTerrainGen2D::BuildSlopesWalls(double width, const tParams& params, cRand& rand, std::vector<float>& out_data)
{
	const double delta_range = std::abs(params[eParamsSlopeDeltaRange]);
	const double delta_min = params[eParamsSlopeDeltaMin];
	const double delta_max = params[eParamsSlopeDeltaMax];

	int beg_idx = static_cast<int>(out_data.size());
	double total_w = BuildWalls(width, params, rand, out_data);
	int end_idx = static_cast<int>(out_data.size());

	OverlaySlopes(delta_range, delta_min, delta_max, 0, beg_idx, end_idx, rand, out_data);

	return total_w;
}

double cTerrainGen2D::BuildSlopesMixed(double width, const tParams& params, cRand& rand, std::vector<float>& out_data)
{
	const double delta_range = std::abs(params[eParamsSlopeDeltaRange]);
	const double delta_min = params[eParamsSlopeDeltaMin];
	const double delta_max = params[eParamsSlopeDeltaMax];

	int beg_idx = static_cast<int>(out_data.size());
	double total_w = BuildMixed(width, params, rand, out_data);
	int end_idx = static_cast<int>(out_data.size());

	OverlaySlopes(delta_range, delta_min, delta_max, 0, beg_idx, end_idx, rand, out_data);

	return total_w;
}

double cTerrainGen2D::BuildSlopesNarrowGaps(double width, const tParams& params, cRand& rand, std::vector<float>& out_data)
{
	const double delta_range = std::abs(params[eParamsSlopeDeltaRange]);
	const double delta_min = params[eParamsSlopeDeltaMin];
	const double delta_max = params[eParamsSlopeDeltaMax];

	int beg_idx = static_cast<int>(out_data.size());
	double total_w = BuildNarrowGaps(width, params, rand, out_data);
	int end_idx = static_cast<int>(out_data.size());

	OverlaySlopes(delta_range, delta_min, delta_max, 0, beg_idx, end_idx, rand, out_data);

	return total_w;

}

double cTerrainGen2D::BuildCliffs(double width, const tParams& params, cRand& rand, std::vector<float>& out_data)
{
	double spacing_min = params[eParamsCliffSpacingMin];
	double spacing_max = params[eParamsCliffSpacingMax];
	double cliff_h0_min = params[eParamsCliffHeight0Min];
	double cliff_h0_max = params[eParamsCliffHeight0Max];
	double cliff_h1_min = params[eParamsCliffHeight1Min];
	double cliff_h1_max = params[eParamsCliffHeight1Max];
	int mini_count_max = static_cast<int>(params[eParamsCliffMiniCountMax]);

	double bump_h_min = params[eParamsBumpHeightMin];
	double bump_h_max = params[eParamsBumpHeightMax];

	const double delta_range = std::abs(params[eParamsSlopeDeltaRange]);
	const double delta_min = params[eParamsSlopeDeltaMin];
	const double delta_max = params[eParamsSlopeDeltaMax];

	int beg_idx = static_cast<int>(out_data.size());

	double total_w = 0;
	while (total_w < width)
	{
		double min_h = 0;
		double max_h = 0;

		bool valid_h0 = (cliff_h0_min != 0 || cliff_h0_max != 0);
		bool valid_h1 = (cliff_h1_min != 0 || cliff_h1_max != 0);
		if (valid_h0 && valid_h1)
		{
			bool heads = rand.FlipCoin();
			min_h = (heads) ? cliff_h0_min : cliff_h1_min;
			max_h = (heads) ? cliff_h0_max : cliff_h1_max;
		}
		else if (valid_h0)
		{
			min_h = cliff_h0_min;
			max_h = cliff_h0_max;
		}
		else
		{
			min_h = cliff_h1_min;
			max_h = cliff_h1_max;
		}

		double w = rand.RandDouble(spacing_min, spacing_max);
		double h = rand.RandDouble(min_h, max_h);

		double curr_w = 0;
		double curr_delta_h = 0;
		int num_mini = rand.RandInt(0, mini_count_max + 1);
		for (int i = 0; i < num_mini + 1; ++i)
		{
			const double mini_w = (i == 0) ? w : 0.1;
			double mini_h = rand.RandDouble(curr_delta_h, h);
			mini_h = (i == num_mini) ? h : mini_h;

			double dh = mini_h - curr_delta_h;
			curr_w += AddStep(mini_w, dh, out_data);

			curr_delta_h = mini_h;
		}

		total_w += curr_w;
	}

	int end_idx = static_cast<int>(out_data.size());
	OverlaySlopes(delta_range, delta_min, delta_max, 0, beg_idx, end_idx, rand, out_data);
	OverlayBumps(bump_h_min, bump_h_max, beg_idx, end_idx, rand, out_data);

	return total_w;
}

int cTerrainGen2D::CalcNumVerts(double w)
{
	int num_verts = static_cast<int>(std::ceil(w / gVertSpacing)) + 1;
	return num_verts;
}

double cTerrainGen2D::AddFlat(double width, std::vector<float>& out_data)
{
	int num_verts = CalcNumVerts(width);
	int buffer_size = static_cast<int>(out_data.size());
	int buffer_size0 = buffer_size;
	bool start_empty = buffer_size0 == 0;

	float base_h = 0;
	if (!start_empty)
	{
		--num_verts;
		base_h = out_data[buffer_size0 - 1];
	}

	for (int i = 0; i < num_verts; ++i)
	{
		out_data.push_back(base_h);
	}

	buffer_size = static_cast<int>(out_data.size());
	int verts_added = buffer_size - buffer_size0;
	if (start_empty)
	{
		--verts_added;
	}
	double width_added = verts_added * gVertSpacing;

	return width_added;
}

double cTerrainGen2D::AddBox(double spacing, double width, double depth, std::vector<float>& out_data)
{
	int num_verts = CalcNumVerts(spacing);
	int buffer_size = static_cast<int>(out_data.size());
	int buffer_size0 = buffer_size;
	bool start_empty = buffer_size0 == 0;

	float base_h = 0;
	if (!start_empty)
	{
		--num_verts;
		base_h = out_data[buffer_size0 - 1];
	}

	out_data.reserve(buffer_size + num_verts);
	for (int i = 0; i < num_verts; ++i)
	{
		out_data.push_back(base_h);
	}

	num_verts = CalcNumVerts(width) - 1;
	buffer_size = static_cast<int>(out_data.size());

	out_data.reserve(buffer_size + num_verts);
	float gap_h = static_cast<float>(base_h + depth);
	for (int i = 0; i < num_verts; ++i)
	{
		out_data.push_back(gap_h);
	}
	out_data.push_back(base_h);

	buffer_size = static_cast<int>(out_data.size());
	int verts_added = buffer_size - buffer_size0;
	if (start_empty)
	{
		--verts_added;
	}
	double width_added = verts_added * gVertSpacing;

	return width_added;
}

double cTerrainGen2D::AddStep(double width, double height, std::vector<float>& out_data)
{
	int num_verts = CalcNumVerts(width);
	int buffer_size = static_cast<int>(out_data.size());
	int buffer_size0 = buffer_size;
	bool start_empty = buffer_size0 == 0;

	float base_h = 0;
	if (!start_empty)
	{
		--num_verts;
		base_h = out_data[buffer_size0 - 1];
	}

	out_data.reserve(buffer_size + num_verts + 1);
	for (int i = 0; i < num_verts; ++i)
	{
		out_data.push_back(base_h);
	}
	out_data.push_back(static_cast<float>(base_h + height));

	buffer_size = static_cast<int>(out_data.size());
	int verts_added = buffer_size - buffer_size0;
	if (start_empty)
	{
		--verts_added;
	}
	double width_added = verts_added * gVertSpacing;

	return width_added;
}

double cTerrainGen2D::AddSlope(double width, double slope, std::vector<float>& out_data)
{
	int num_verts = CalcNumVerts(width) - 1;
	int buffer_size = static_cast<int>(out_data.size());
	int buffer_size0 = buffer_size;
	bool start_empty = buffer_size0 == 0;

	float base_h = 0;
	if (start_empty)
	{
		out_data.push_back(base_h);
	}
	else
	{
		base_h = out_data[buffer_size0 - 1];
	}

	out_data.reserve(buffer_size + num_verts);
	for (int i = 0; i < num_verts; ++i)
	{
		double d = (i + 1) * (width / num_verts);
		out_data.push_back(static_cast<float>(base_h + slope * d));
	}

	buffer_size = static_cast<int>(out_data.size());
	int verts_added = buffer_size - buffer_size0;
	if (start_empty)
	{
		--verts_added;
	}
	double width_added = verts_added * gVertSpacing;

	return width_added;
}

void cTerrainGen2D::OverlaySlopes(double delta_range, double delta_min, double delta_max, double init_slope, int beg_idx, int end_idx,
									cRand& rand, std::vector<float>& out_data)
{
	double curr_slope = init_slope;
	double curr_delta_h = 0;
	assert(beg_idx < out_data.size());
	assert(end_idx <= out_data.size());
	assert(delta_min < delta_max);

	double delta_mean = 0.5 * (delta_min + delta_max);
	double delta_diff = 0.5 * (delta_max - delta_min);

	for (int i = beg_idx; i < end_idx; ++i)
	{
		const double w = 0.1;
		double delta = rand.RandDouble(0, delta_range);

		double sign_rand = rand.RandDouble(-1, 1);
		double sign_threshold = (curr_slope - delta_mean) / delta_diff;
		bool neg = sign_rand < sign_threshold;
		delta = (neg) ? -delta : delta;

		curr_slope += delta;
		curr_delta_h += curr_slope * gVertSpacing;

		out_data[i] += static_cast<float>(curr_delta_h);
	}
}

void cTerrainGen2D::OverlayBumps(double min_delta_h, double max_delta_h, int beg_idx, int end_idx,
									cRand& rand, std::vector<float>& out_data)
{
	double curr_slope = 0;
	double curr_delta_h = 0;
	assert(beg_idx < out_data.size());
	assert(end_idx <= out_data.size());

	for (int i = beg_idx; i < end_idx - 1; ++i)
	{
		double delta = rand.RandSign() * rand.RandDouble(min_delta_h, max_delta_h);
		out_data[i] += static_cast<float>(delta);
	}
}
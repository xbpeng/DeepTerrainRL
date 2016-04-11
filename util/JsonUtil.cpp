#include "JsonUtil.h"

std::string cJsonUtil::BuildVectorJson(const Eigen::VectorXd& vec)
{
	std::string json = "";
	for (int i = 0; i < vec.size(); ++i)
	{
		if (i != 0)
		{
			json += ", ";
		}
		json += std::to_string(vec[i]);
	}
	json = "[" + json + "]";
	return json;
}

bool cJsonUtil::ReadVectorJson(const Json::Value& root, Eigen::VectorXd& out_vec)
{
	bool succ = false;
	int num_vals = root.size();
	
	if (root.isArray())
	{
		out_vec.resize(num_vals);
		for (int i = 0; i < num_vals; ++i)
		{
			Json::Value json_elem = root.get(i, 0);
			out_vec[i] = json_elem.asDouble();
		}
		succ = true;
	}

	return succ;
}
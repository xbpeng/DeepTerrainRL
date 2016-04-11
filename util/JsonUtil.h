#pragma once

#include <string>
#include <json/json.h>
#include "util/MathUtil.h"

class cJsonUtil
{
public:
	static std::string BuildVectorJson(const Eigen::VectorXd& vec);
	static bool ReadVectorJson(const Json::Value& root, Eigen::VectorXd& out_vec);

private:
	
};

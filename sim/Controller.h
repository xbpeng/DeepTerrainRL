#pragma once

#include <memory>
#include "util/MathUtil.h"

class cSimCharacter;

class cController : public std::enable_shared_from_this<cController>
{
public:
	enum eMode
	{
		eModeActive,
		eModeInactive,
		eModePassive,
		eModeMax
	};

	virtual ~cController();

	virtual void Init(cSimCharacter* character);
	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_step);
	virtual bool IsValid() const;

	virtual void BuildOptParams(Eigen::VectorXd& out_params) const;
	virtual void SetOptParams(const Eigen::VectorXd& params);
	virtual void SetOptParams(const Eigen::VectorXd& params, Eigen::VectorXd& out_params) const;
	virtual int GetNumOptParams() const;
	virtual void FetchOptParamScale(Eigen::VectorXd& out_scale) const;
	virtual void OutputOptParams(const std::string& file, const Eigen::VectorXd& params) const;
	virtual void OutputOptParams(FILE* f, const Eigen::VectorXd& params) const;
	virtual void ReadParams(const std::string& file);
	virtual void ReadParams(std::ifstream& f_stream);

	virtual void SetActive(bool active);
	virtual bool IsActive() const;
	virtual void SetMode(eMode mode);

protected:
	cSimCharacter* mChar;
	eMode mMode;
	bool mValid;

	cController();
};
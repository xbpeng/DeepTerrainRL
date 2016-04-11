#pragma once

#include <json/json.h>
#include "KinTree.h"

class cMotion
{
public:
	enum eFrameParams
	{
		eFrameTime,
		eFrameMax
	};

	typedef Eigen::VectorXd tFrame;

	cMotion();
	virtual ~cMotion();

	virtual void Clear();
	virtual bool Load(const std::string& file);
	virtual bool IsValid() const;

	virtual int GetNumFrames() const;
	virtual int GetNumDof() const;
	virtual tFrame GetFrame(int i) const;
	virtual tFrame BlendFrames(int a, int b, double lerp) const;

	virtual tFrame CalcFrame(double time) const;
	virtual double GetDuration() const;

	virtual int CalcCycleCount(double time) const;
	virtual void CalcIndexPhase(double time, int& out_idx, double& out_phase) const;

protected:
	bool mLoop;
	Eigen::MatrixXd mFrames;

	virtual bool LoadJson(const Json::Value& root);
	virtual bool ParseFrameJson(const Json::Value& root, Eigen::VectorXd& out_frame) const;

	virtual double GetFrameTime(int i) const;
	virtual void PostProcessFrames(Eigen::MatrixXd& frames) const;
	virtual int GetFrameSize() const;
};

#pragma once
#include <memory>

#include "DrawScenarioSimChar.h"

class cDrawScenarioPoliEval : public cDrawScenarioSimChar
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioPoliEval(cCamera& cam);
	virtual ~cDrawScenarioPoliEval();

	virtual void Init();
	virtual void ParseArgs(const cArgParser& parser);
	virtual void Keyboard(unsigned char key, int x, int y);

	virtual std::string BuildTextInfoStr() const;

protected:
	unsigned long mRandSeed;

	virtual void BuildScene();
	virtual void ResetCallback();

	virtual void CommandAction(int a);
};
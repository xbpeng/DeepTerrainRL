#pragma once
#include <memory>

#include "DrawScenario.h"
#include "ScenarioSimChar.h"

class cDrawScenarioSimInteractive : public cDrawScenario
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	virtual ~cDrawScenarioSimInteractive();

	virtual void Init();
	virtual void Update(double time_elapsed);

	virtual void Reset();
	virtual void Clear();
	virtual void MouseClick(int button, int state, double x, double y);
	virtual void MouseMove(double x, double y);

protected:
	cDrawScenarioSimInteractive(cCamera& cam);

	// UI stuff
	tVector mClickScreenPos;
	tVector mDragScreenPos;
	tVector mSelectObjLocalPos;
	cSimObj* mSelectedObj;

	virtual void ResetUI();
	virtual tVector GetDefaultCamFocus() const;

	virtual const std::shared_ptr<cScenarioSimChar>& GetScene() const = 0;
	virtual cSimObj* RayTest(const tVector& start, const tVector& end, tVector& out_intersection);
	virtual bool ObjectSelected() const;
	virtual void ApplyUIForce(double time_step);
};
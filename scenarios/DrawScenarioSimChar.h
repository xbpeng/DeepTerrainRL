#pragma once
#include <memory>
#include <GL/glew.h>

#include "DrawScenarioSimInteractive.h"
#include "ScenarioSimChar.h"
#include "sim/CharTracer.h"

class cShader;
class cSkyBox;
class cPostProcessor;
class cTextureDesc;
class cShadowMap;
class cGBuffer;

class cDrawScenarioSimChar : public cDrawScenarioSimInteractive
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	cDrawScenarioSimChar(cCamera& cam);
	virtual ~cDrawScenarioSimChar();

	virtual void Init();
	virtual void ParseArgs(const cArgParser& parser);

	virtual void Reset();
	virtual void Clear();
	virtual void Update(double time_elapsed);
	virtual void Keyboard(unsigned char key, int x, int y);
	virtual void MouseClick(int button, int state, double x, double y);
	virtual void MouseMove(double x, double y);
	virtual void Reshape(int w, int h);

	virtual std::string BuildTextInfoStr() const;
	virtual void Shutdown();

	std::string GetName() const;

protected:
	std::shared_ptr<cScenarioSimChar> mScene;
	cArgParser mArgParser;
	bool mDrawInfo;
	bool mDrawPoliInfo;
	bool mDrawFeatures;
	bool mDrawPolicyPlots;

	bool mEnableTrace;
	cCharTracer mTracer;
	std::vector<int> mTraceHandles;
	tVector mCamDelta;

	virtual void BuildScene();
	virtual tVector GetCamTrackPos() const;
	virtual tVector GetCamStillPos() const;

	virtual void UpdateTracer(double time_elapsed);

	virtual void InitTracer();
	virtual int AddCharTrace(const std::shared_ptr<cSimCharacter>& character,
								const tVectorArr& cols);
	virtual void ToggleTrace();

	virtual void DrawScene();
	virtual void DrawGrid() const;
	virtual void DrawGroundMainScene();
	virtual void DrawCharacterMainScene();
	virtual void DrawObjsMainScene();
	virtual void DrawGround() const;
	virtual void DrawCharacter() const;
	virtual void DrawTrace() const;
	virtual void DrawObjs() const;

	virtual tVector GetVisOffset() const;
	virtual tVector GetLineColor() const;
	virtual tVector GetGroundColor() const;

	virtual void DrawInfo() const;
	virtual void DrawCoM() const;
	virtual void DrawTorque() const;
	virtual void DrawPerturbs() const;
	virtual void DrawCtrlInfo() const;
	virtual void DrawPoliInfo() const;
	virtual void DrawFeatures() const;
	virtual void DrawPolicyPlots() const;

	virtual const std::shared_ptr<cScenarioSimChar>& GetScene() const;

	virtual std::string GetOutputCharFile() const;
	virtual void OutputCharState(const std::string& out_file) const;

	virtual void SpawnProjectile();
	virtual void SpawnBigProjectile();
};
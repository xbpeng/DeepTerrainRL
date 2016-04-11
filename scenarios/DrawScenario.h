#pragma once
#include <memory>

#include "Scenario.h"
#include "render/Camera.h"

class cDrawScenario : public cScenario
{
public:
	enum eCamTrackMode
	{
		eCamTrackModeX,
		eCamTrackModeY,
		eCamTrackModeXYZ,
		eCamTrackModeStill,
		eCamTrackModeFixed,
		eCamTrackModeMax
	};

	virtual ~cDrawScenario();

	virtual void ParseArgs(const cArgParser& parser);
	virtual void Update(double time_elapsed);

	virtual void Reset();
	virtual void Draw();
	virtual void Keyboard(unsigned char key, int x, int y);
	virtual void MouseClick(int button, int state, double world_x, double world_y);
	virtual void MouseMove(double x, double y);
	virtual void Reshape(int w, int h);

	virtual eCamTrackMode GetTrackMode() const;
	virtual eCamTrackMode GetTrackMode0() const;

	virtual std::string BuildTextInfoStr() const;

protected:
	cCamera& mCam;
	eCamTrackMode mCamTrackMode;
	eCamTrackMode mCamTrackMode0;

	cDrawScenario(cCamera& cam);
	virtual void ParseCamTrackMode(const cArgParser& parser, eCamTrackMode& out_mode) const;

	virtual void UpdateCamera();
	virtual void UpdateCameraTracking();
	virtual void UpdateCameraStill();
	virtual double GetCamStillSnapDistX() const;

	virtual tVector GetCamTrackPos() const;
	virtual tVector GetCamStillPos() const;
	virtual void ToggleCamTrackMode(eCamTrackMode mode);

	virtual void ResetCamera();
	virtual tVector GetDefaultCamFocus() const;

	virtual void DrawSetup();
	virtual void DrawCleanup();
	virtual void DrawScene();
};
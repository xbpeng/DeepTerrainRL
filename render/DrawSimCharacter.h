#pragma once

#include "sim/SimCharacter.h"
#include "sim/Ground.h"
#include "util/CircularBuffer.h"
#include "render/Camera.h"

class cCharController;
class cDogController;
class cDogControllerCacla;
class cRaptorController;
class cRaptorControllerCacla;

class cDrawSimCharacter
{
public:
	static void Draw(const cSimCharacter& character, const tVector& fill_tint, const tVector& line_col, bool enable_draw_shape = false);
	static void DrawCoM(const cSimCharacter& character, double marker_size, double vel_scale,
						const tVector& col, const tVector& offset);
	static void DrawTorque(const cSimCharacter& character, const tVector& offset);
	static void DrawCtrlInfo(const cCharController* ctrl, const tVector& offset);
	static void DrawPoliInfo(const cCharController* ctrl, const cCamera& cam);

	static void DrawCharFeatures(const cSimCharacter& character, const cGround& ground, double marker_size, double vel_scale,
								const tVector& pos_col, const tVector& vel_col, const tVector& offset);
	static void DrawTerainFeatures(const cSimCharacter& character, double marker_size,
									const tVector& terrain_col, const tVector& offset);

	static void DrawPolicyPlots(const cCharController* ctrl, const cCamera& cam);

protected:
	static void DrawInfoValLog(const cCircularBuffer<double>& val_log, double aspect);

	static int GetCharNumGroundFeatures(const cSimCharacter& character);
	static tVector GetCharGroundSample(const cSimCharacter& character, int i);
	static tVector GetCharGroundSampleOrigin(const cSimCharacter& character);

	static void DrawSimBody(const cSimCharacter& character, const tVector& fill_tint, const tVector& line_col);
	static void DrawShapes(const cSimCharacter& character, const tVector& fill_tint, const tVector& line_col);

};

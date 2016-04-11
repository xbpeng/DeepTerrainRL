#include "DrawSimCharacter.h"
#include "DrawCharacter.h"
#include "sim/DogController.h"
#include "sim/DogControllerCacla.h"
#include "sim/DogControllerMACE.h"
#include "sim/GoatControllerMACE.h"
#include "sim/RaptorControllerCacla.h"
#include "sim/RaptorControllerMACE.h"
#include "sim/RaptorControllerMACE.h"
#include "sim/SimBox.h"
#include "render/DrawObj.h"
#include "render/DrawPerturb.h"
#include "render/GraphUtil.h"

void cDrawSimCharacter::Draw(const cSimCharacter& character, const tVector& fill_tint, const tVector& line_col, bool enable_draw_shape)
{
	bool has_draw_shapes = character.HasDrawShapes();
	if (has_draw_shapes && enable_draw_shape)
	{
		DrawShapes(character, fill_tint, line_col);
	}
	else
	{
		DrawSimBody(character, fill_tint, line_col);
	}
}

void cDrawSimCharacter::DrawCoM(const cSimCharacter& character, double marker_size, double vel_scale, 
								const tVector& col, const tVector& offset)
{
	const double arrow_size = marker_size * 0.65;
	tVector com = character.CalcCOM();
	tVector com_vel = character.CalcCOMVel();
	
	cDrawUtil::SetLineWidth(4);
	cDrawUtil::SetColor(tVector(col[0], col[1], col[2], col[3]));
	cDrawUtil::DrawCross(com + offset, marker_size);
	cDrawUtil::DrawArrow2D(com + offset, com + offset + com_vel * vel_scale, arrow_size);
}

void cDrawSimCharacter::DrawTorque(const cSimCharacter& character, const tVector& offset)
{
	int num_joints = character.GetNumJoints();
	for (int j = 0; j < num_joints; ++j)
	{
		const cJoint& joint = character.GetJoint(j);
		if (joint.IsValid())
		{
			const tVector& torque = joint.GetTorque();
			tVector pos = joint.GetPos();
			cDrawPerturb::DrawTorque2D(pos + offset, torque);
		}
	}
}

void cDrawSimCharacter::DrawCtrlInfo(const cCharController* ctrl, const tVector& offset)
{
	if (ctrl != nullptr)
	{
		glPushMatrix();
		cDrawUtil::Translate(offset);

		int num_ground_samples = ctrl->GetNumGroundSamples();
		cDrawUtil::SetColor(tVector(1, 0, 0, 0.5));
		for (int s = 0; s < num_ground_samples; ++s)
		{
			tVector sample_pos = ctrl->GetGroundSample(s);

			glPushMatrix();
			cDrawUtil::Translate(sample_pos);
			cDrawUtil::DrawDisk(0.02, 16);
			glPopMatrix();
		}

		glPopMatrix();
	}
}

void cDrawSimCharacter::DrawPoliInfo(const cCharController* ctrl, const cCamera& cam)
{
#if defined(ENABLE_DEBUG_VISUALIZATION)
	const cTerrainRLCharController* trl_ctrl = dynamic_cast<const cTerrainRLCharController*>(ctrl);
	if (trl_ctrl != nullptr)
	{
		const cCircularBuffer<double>& val_log = trl_ctrl->GetPoliValLog();
		double aspect = cam.GetAspectRatio();
		DrawInfoValLog(val_log, aspect);
	}
#endif
}

void cDrawSimCharacter::DrawCharFeatures(const cSimCharacter& character, const cGround& ground, double marker_size, double vel_scale,
										const tVector& pos_col, const tVector& vel_col, const tVector& offset)
{
	const double arrow_size = marker_size * 0.65;
	tVector root_pos = character.GetRootPos();
	double ground_h = ground.SampleHeight(root_pos);
	tVector ground_pos = root_pos;
	ground_pos[1] = ground_h;

	cDrawUtil::SetColor(tVector(pos_col[0], pos_col[1], pos_col[2], pos_col[3]));
	cDrawUtil::DrawArrow2D(ground_pos + offset, root_pos + offset, arrow_size);

	for (int i = 0; i < character.GetNumBodyParts(); ++i)
	{
		if (character.IsValidBodyPart(i))
		{
			const auto& curr_part = character.GetBodyPart(i);
			tVector pos = curr_part->GetPos();
			tVector vel = curr_part->GetLinearVelocity();

			cDrawUtil::SetColor(tVector(pos_col[0], pos_col[1], pos_col[2], pos_col[3]));
			cDrawUtil::DrawArrow2D(root_pos + offset, pos + offset, arrow_size);
			cDrawUtil::SetColor(tVector(vel_col[0], vel_col[1], vel_col[2], vel_col[3]));
			cDrawUtil::DrawArrow2D(pos + offset, pos + vel * vel_scale + offset, arrow_size);
		}
	}
}

void cDrawSimCharacter::DrawTerainFeatures(const cSimCharacter& character, double marker_size,
											const tVector& terrain_col, const tVector& offset)
{
	const double arrow_size = marker_size * 0.65;
	int num_terrain_features = GetCharNumGroundFeatures(character);

 	tVector origin = GetCharGroundSampleOrigin(character);
	double base_h = origin[1];

	cDrawUtil::SetColor(tVector(terrain_col[0], terrain_col[1], terrain_col[2], terrain_col[3]));
	for (int i = 0; i < num_terrain_features; ++i)
	{
		tVector sample = GetCharGroundSample(character, i);
		
		tVector base_sample = sample;
		base_sample[1] = base_h;
		
		cDrawUtil::DrawArrow2D(base_sample + offset, sample + offset, arrow_size);
	}
}

void cDrawSimCharacter::DrawPolicyPlots(const cCharController* ctrl, const cCamera& cam)
{
	const tVector& char_feature_col = tVector(1, 0, 0, 1);
	const tVector& terr_feature_col = tVector(0, 0, 1, 1);
	const tVector& action_feature_col = tVector(0, 0.5, 0, 1);
	const tVector& action_val_col = tVector(0, 0.5, 0, 1);

	double aspect = cam.GetAspectRatio();

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	const double h = 0.4;
	const double w = 16.0 / 9 * h / aspect;
	const double x_offset = -w * 1.05;
	const double y_offset = -h * 1.05;

	const double char_min_val = -2;
	const double char_max_val = 2;
	const double char_base_val = 0;

	Eigen::VectorXd char_features;
#if defined(ENABLE_DEBUG_VISUALIZATION)
	ctrl->GetVisCharacterFeatures(char_features);
#endif

	tVector char_features_plot_size = tVector(w, h, 0, 0);
	tVector char_features_plot_pos = tVector(1 + x_offset, 1 + y_offset, -1, 0);
	char_features_plot_pos += 0.5 * char_features_plot_size;
	char_features_plot_pos[1] += y_offset;

	cGraphUtil::tBarPlot char_features_plot;
	char_features_plot.mMinVal = char_min_val;
	char_features_plot.mMaxVal = char_max_val;
	char_features_plot.mBaseVal = char_base_val;
	char_features_plot.mVals = char_features;
	char_features_plot.mColors.push_back(char_feature_col);
	cGraphUtil::DrawBarPlot(char_features_plot, char_features_plot_pos, char_features_plot_size);

	
	const double terr_min_val = -2;
	const double terr_max_val = 2;
	const double terr_base_val = 0;

	Eigen::VectorXd terr_features;
#if defined(ENABLE_DEBUG_VISUALIZATION)
	ctrl->GetVisTerrainFeatures(terr_features);
#endif
	tVector terr_features_plot_size = char_features_plot_size;
	tVector terr_features_plot_pos = char_features_plot_pos;
	terr_features_plot_pos[0] += x_offset;

	cGraphUtil::tBarPlot terr_features_plot;
	terr_features_plot.mMinVal = terr_min_val;
	terr_features_plot.mMaxVal = terr_max_val;
	terr_features_plot.mBaseVal = terr_base_val;
	terr_features_plot.mVals = terr_features;
	terr_features_plot.mColors.push_back(terr_feature_col);
	cGraphUtil::DrawBarPlot(terr_features_plot, terr_features_plot_pos, terr_features_plot_size);


	const double action_min_val = -1;
	const double action_max_val = 1;
	const double action_base_val = 0;

	Eigen::VectorXd action_features;
#if defined(ENABLE_DEBUG_VISUALIZATION)
	ctrl->GetVisActionFeatures(action_features);
#endif
	tVector action_features_plot_size = char_features_plot_size;
	tVector action_features_plot_pos = char_features_plot_pos;
	action_features_plot_pos[1] += y_offset;

	cGraphUtil::tBarPlot action_features_plot;
	action_features_plot.mMinVal = action_min_val;
	action_features_plot.mMaxVal = action_max_val;
	action_features_plot.mBaseVal = action_base_val;
	action_features_plot.mVals = action_features;
	action_features_plot.mColors.push_back(action_feature_col);
	cGraphUtil::DrawBarPlot(action_features_plot, action_features_plot_pos, action_features_plot_size);


	const double val_min_val = 0;
	const double val_max_val = 1;
	const double val_base_val = 0;

	Eigen::VectorXd action_vals;
#if defined(ENABLE_DEBUG_VISUALIZATION)
	ctrl->GetVisActionValues(action_vals);
#endif
	if (action_vals.size() > 0)
	{
		tVector val_plot_size = action_features_plot_size;
		tVector val_plot_pos = action_features_plot_pos;
		val_plot_pos[0] += x_offset;

		cGraphUtil::tBarPlot val_plot;
		val_plot.mMinVal = val_min_val;
		val_plot.mMaxVal = val_max_val;
		val_plot.mBaseVal = val_base_val;
		val_plot.mVals = action_vals;
		val_plot.mColors.push_back(tVector(0, 0, 1, 0.5));
		val_plot.mColors.push_back(tVector(1, 0, 0, 0.5));
		val_plot.mColors.push_back(tVector(0, 0.5, 0, 0.5));
		val_plot.mColors.push_back(tVector(0.75, 0, 0.75, 0.5));
		val_plot.mColors.push_back(tVector(0, 0.5, 0.5, 0.5));
		val_plot.mColors.push_back(tVector(0, 0, 0, 0.5));
		cGraphUtil::DrawBarPlot(val_plot, val_plot_pos, val_plot_size);
	}

	glMatrixMode(GL_PROJECTION);
	glPopMatrix();

	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

void cDrawSimCharacter::DrawInfoValLog(const cCircularBuffer<double>& val_log, double aspect)
{
	const double min_val = 0;
	const double max_val = 1;
	
	int num_val = static_cast<int>(val_log.GetSize());
	if (num_val > 0)
	{
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glLoadIdentity();

		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glLoadIdentity();

		const double h = 0.4;
		const double w = 16.0 / 9 * h / aspect;

		tVector origin = tVector::Zero();
		origin[0] = 1 - w * 1.05;
		origin[1] = 1 - h * 1.05;
		origin[2] = -1;

		int capacity = static_cast<int>(val_log.GetCapacity());

		double prev_val = val_log[0];
		cDrawUtil::SetLineWidth(1);
		cDrawUtil::SetColor(tVector(1, 1, 1, 0.5));
		cDrawUtil::DrawRect(origin + 0.5 * tVector(w, h, 0, 0), tVector(w, h, 0, 0));
		cDrawUtil::SetColor(tVector(0, 0, 0, 1));
		cDrawUtil::DrawRect(origin + 0.5 * tVector(w, h, 0, 0), tVector(w, h, 0, 0), cDrawUtil::eDrawWire);

		cDrawUtil::SetLineWidth(1);
		cDrawUtil::SetPointSize(2);
		cDrawUtil::SetColor(tVector(1, 0, 0, 0.5));

		for (int i = 1; i < num_val; ++i)
		{
			double curr_val = val_log[i];

			tVector a = tVector::Zero();
			tVector b = tVector::Zero();

			a[0] = w * (i - 1.0) / (capacity - 1.0);
			b[0] = w * (i) / (capacity - 1.0);

			a[1] = h * cMathUtil::Clamp((prev_val - min_val) / (max_val - min_val), 0.0, 1.0);
			b[1] = h * cMathUtil::Clamp((curr_val - min_val) / (max_val - min_val), 0.0, 1.0);

			a += origin;
			b += origin;

			cDrawUtil::DrawLine(a, b);
			cDrawUtil::DrawPoint(b);
			prev_val = curr_val;
		}

		glMatrixMode(GL_PROJECTION);
		glPopMatrix();

		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();
	}
}

int cDrawSimCharacter::GetCharNumGroundFeatures(const cSimCharacter& character)
{
	const auto& ctrl = character.GetController();
	int num_samples = 0;
	if (ctrl != nullptr)
	{
		num_samples = ctrl->GetNumGroundSamples();
	}
	
	return num_samples;
}

tVector cDrawSimCharacter::GetCharGroundSample(const cSimCharacter& character, int i)
{
	const auto& ctrl = character.GetController();
	tVector sample = tVector::Zero();
	if (ctrl != nullptr)
	{
		sample = ctrl->GetGroundSample(i);
	}

	return sample;
}

tVector cDrawSimCharacter::GetCharGroundSampleOrigin(const cSimCharacter& character)
{
	const auto& ctrl = character.GetController();
	tVector origin = tVector::Zero();
	if (ctrl != nullptr)
	{
		origin = ctrl->GetGroundSampleOrigin();
	}

	return origin;
}

void cDrawSimCharacter::DrawSimBody(const cSimCharacter& character, const tVector& fill_tint, const tVector& line_col)
{
	const tVector gContactCol = tVector(0.5, 0.75, 0.5, 1);

	cDrawUtil::SetLineWidth(1);
	for (int i = 0; i < character.GetNumBodyParts(); ++i)
	{
		if (character.IsValidBodyPart(i))
		{
			const std::shared_ptr<cSimBox>& curr_part = std::static_pointer_cast<cSimBox>(character.GetBodyPart(i));
			tVector pos = curr_part->GetPos();

			tVector col;
			if (curr_part->IsInContact())
			{
				col = gContactCol;
			}
			else
			{
				col = character.GetPartColor(i);
				col = col.cwiseProduct(fill_tint);
			}

			cDrawUtil::SetColor(col);
			cDrawObj::Draw(curr_part.get(), cDrawUtil::eDrawSolid);

			if (line_col[3] > 0)
			{
				cDrawUtil::SetColor(line_col);
				cDrawObj::Draw(curr_part.get(), cDrawUtil::eDrawWire);
			}
		}
	}
}

void cDrawSimCharacter::DrawShapes(const cSimCharacter& character, const tVector& fill_tint, const tVector& line_col)
{
	assert(character.HasDrawShapes());
	const auto& shape_defs = character.GetDrawShapeDefs();
	size_t num_shapes = shape_defs.rows();

	cDrawUtil::SetLineWidth(1);
	for (int i = 0; i < num_shapes; ++i)
	{
		cKinTree::tDrawShapeDef curr_def = shape_defs.row(i);
		cDrawCharacter::DrawShape(character, curr_def, fill_tint, line_col);
	}
}
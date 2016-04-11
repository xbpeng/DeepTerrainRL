#include "CharTracer.h"
#include "render/DrawUtil.h"

const int gNumEndEffMarkers = 4;
const double gMarkerSize = 0.075;

cCharTracer::tParams::tParams()
{
	mType = eTraceJoint;
	mChar = nullptr;
	mTraceID = 0;
	mColIdx = 0;
	mColors.resize(1);
	mColors[0] = tVector(0, 0, 1, 0.5);
}

bool cCharTracer::tParams::IsValid() const
{
	bool valid = true;

	if (mChar == nullptr)
	{
		valid = false;
	}
	else
	{
		if (mType == eTraceJoint || mType == eTracePart)
		{
			if (mTraceID < 0 || mTraceID >= mChar->GetNumJoints())
			{
				valid = false;
			}
			else if (mType == eTracePart)
			{
				if (!mChar->IsValidBodyPart(mTraceID))
				{
					valid = false;
				}
			}
		}
	}
	
	return valid;
}

cCharTracer::cCharTracer()
{
	mBufferSize = 300;
	mSamplePeriod = 0.066;
}

cCharTracer::~cCharTracer()
{

}

void cCharTracer::Init(int buffer_size, double sample_period)
{
	Clear();
	mBufferSize = buffer_size;
	mSamplePeriod = sample_period;
}

void cCharTracer::Update(double time_step)
{
	mTimer += time_step;

	for (int i = 0; i < GetNumTraces(); ++i)
	{
		UpdateTrace(mTraces[i]);
	}

	if (mTimer >= mSamplePeriod)
	{
		ResetTimer();
	}
}

void cCharTracer::Reset()
{
	ResetTimer();
	
	for (int i = 0; i < GetNumTraces(); ++i)
	{
		ResetTrace(mTraces[i]);
	}
}

void cCharTracer::Clear()
{
	ResetTimer();
	mTraces.clear();
}

int cCharTracer::AddTrace(const tParams& params)
{
	int handle = gInvalidIdx;
	if (params.IsValid())
	{
		tTrace trace;
		BuildTrace(params, trace);

		handle = GetNumTraces();
		mTraces.push_back(trace);
	}
	else
	{
		assert(false); // invalid trace parameters
	}
	return handle;
}

void cCharTracer::SetTraceColIdx(int handle, int col_idx)
{
	tTrace& trace = GetTrace(handle);
	SetTraceColIdx(col_idx, trace);
}

int cCharTracer::GetNumTraces() const
{
	return static_cast<int>(mTraces.size());
}

void cCharTracer::Draw() const
{
	for (int i = 0; i < GetNumTraces(); ++i)
	{
		DrawTrace(mTraces[i]);
	}
}

void cCharTracer::ResetTimer()
{
	mTimer = 0;
}

void cCharTracer::BuildTrace(const tParams& params, tTrace& out_trace) const
{
	out_trace.mParams = params;
	out_trace.mPosTraj.Reserve(mBufferSize);

	size_t num_contacts = params.mContactList.size();
	size_t end_pos_size = static_cast<size_t>(mBufferSize * mSamplePeriod * num_contacts);
	out_trace.mEndEffPos.Reserve(end_pos_size);

	const auto& character = params.mChar;
	for (int i = 0; i < static_cast<int>(num_contacts); ++i)
	{
		assert(character->IsValidBodyPart(i));
		out_trace.mEndContact.push_back(false);
	}
}

void cCharTracer::ResetTrace(tTrace& out_trace) const
{
	out_trace.mPosTraj.Clear();
	out_trace.mEndEffPos.Clear();
	
	for (size_t e = 0; e < out_trace.mEndContact.size(); ++e)
	{
		out_trace.mEndContact[e] = false;
	}

	SetTraceColIdx(0, out_trace);
}

void cCharTracer::UpdateTrace(tTrace& out_trace) const
{
	UpdateTraceTraj(out_trace);
	UpdateTraceEndPos(out_trace);
}

void cCharTracer::UpdateTraceTraj(tTrace& out_trace) const
{
	if (mTimer >= mSamplePeriod)
	{
		tVector pos = CalcTracePos(out_trace);
		pos[3] = out_trace.mParams.mColIdx;
		out_trace.mPosTraj.Add(pos);
	}
}

void cCharTracer::UpdateTraceEndPos(tTrace& out_trace) const
{
	const auto& character = out_trace.mParams.mChar;
	const std::vector<int>& contact_list = out_trace.mParams.mContactList;
	int num_contacts = static_cast<int>(contact_list.size());

	for (int e = 0; e < num_contacts; ++e)
	{
		int id = contact_list[e];
		bool contact = character->IsInContact(id);
		bool prev_contact = out_trace.mEndContact[e];
		if (contact && !prev_contact)
		{
			tVector pos = character->GetContactPt(id);
			pos[3] = out_trace.mParams.mColIdx;

			tEndEffectorPos end_eff_pos;
			end_eff_pos.mIdx = e;
			end_eff_pos.mPos = pos;

			out_trace.mEndEffPos.Add(end_eff_pos);
		}

		out_trace.mEndContact[e] = contact;
	}
}

tVector cCharTracer::CalcTracePos(const tTrace& trace) const
{
	tVector pos = tVector::Zero();
	const auto& character = trace.mParams.mChar;

	switch (trace.mParams.mType)
	{
	case eTraceCOM:
		pos = character->CalcCOM();
		break;
	case eTraceJoint:
		pos = character->CalcJointPos(trace.mParams.mTraceID);
		break;
	case eTracePart:
		pos = character->GetBodyPart(trace.mParams.mTraceID)->GetPos();
		break;
	default:
		assert(false); // unsupported trace type
		break;
	}

	return pos;
}

const cCharTracer::tTrace& cCharTracer::GetTrace(int handle) const
{
	assert(handle >= 0 && handle < GetNumTraces());
	return mTraces[handle];
}

cCharTracer::tTrace& cCharTracer::GetTrace(int handle)
{
	assert(handle >= 0 && handle < GetNumTraces());
	return mTraces[handle];
}

void cCharTracer::DrawTrace(const tTrace& trace) const
{
	cDrawUtil::SetLineWidth(3);
	DrawTraceTraj(trace);
	DrawTraceEndPos(trace);
}

void cCharTracer::DrawTraceTraj(const tTrace& trace) const
{
	if (trace.mPosTraj.GetSize() > 1)
	{
		for (size_t i = 0; i < trace.mPosTraj.GetSize() - 1; ++i)
		{
			const tVector& vert0 = trace.mPosTraj[i];
			const tVector& vert1 = trace.mPosTraj[i + 1];
			int curr_col_idx = static_cast<int>(vert0[3]);

			assert(curr_col_idx < trace.mParams.mColors.size());
			const tVector& col = trace.mParams.mColors[curr_col_idx];
			cDrawUtil::SetColor(col);
			cDrawUtil::DrawLine(vert0, vert1);
		}
	}
}

void cCharTracer::DrawTraceEndPos(const tTrace& trace) const
{
	const double marker_size = gMarkerSize;

	for (int i = 0; i < static_cast<int>(trace.mEndEffPos.GetSize()); ++i)
	{
		const tEndEffectorPos& end_pos = trace.mEndEffPos[i];
		int idx = end_pos.mIdx;
		const tVector & pos = end_pos.mPos;

		int curr_col_idx = static_cast<int>(pos[3]);
		assert(curr_col_idx < trace.mParams.mColors.size());
		const tVector& col = trace.mParams.mColors[curr_col_idx];
		cDrawUtil::SetColor(col);

		int draw_idx = idx % gNumEndEffMarkers;

		switch (draw_idx)
		{
		case 0:
			cDrawUtil::DrawRect(pos, tVector(marker_size, marker_size, 0, 0));
			break;
		case 1:
			cDrawUtil::DrawTriangle(pos, marker_size);
			break;
		case 2:
			cDrawUtil::DrawCross(pos, marker_size);
			break;
		case 3:
			cDrawUtil::DrawDisk(pos, marker_size, 16);
			break;
		default:
			assert(false);
			break;
		}
	}
}

void cCharTracer::SetTraceColIdx(int idx, tTrace& out_trace) const
{
	int num_cols = static_cast<int>(out_trace.mParams.mColors.size());
	idx = idx % num_cols;
	out_trace.mParams.mColIdx = idx;
}
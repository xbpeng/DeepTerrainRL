#pragma once
#include <memory>
#include "util/CircularBuffer.h"
#include "sim/SimCharacter.h"

class cCharTracer
{
public:
	enum eTraceType
	{
		eTraceCOM,
		eTraceJoint,
		eTracePart,
		eTrceMax
	};

	struct tParams
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		eTraceType mType;
		int mColIdx;
		std::shared_ptr<cSimCharacter> mChar;
		std::vector<int> mContactList;
		int mTraceID;
		tVectorArr mColors;

		tParams();
		bool IsValid() const;
	};

	cCharTracer();
	virtual ~cCharTracer();

	virtual void Init(int buffer_size, double sample_period);
	virtual void Update(double time_step);
	virtual void Reset();
	virtual void Clear();

	virtual int AddTrace(const tParams& params);
	virtual int GetNumTraces() const;
	virtual void SetTraceColIdx(int handle, int col_idx);

	virtual void Draw() const;

protected:
	struct tEndEffectorPos
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		int mIdx;
		tVector mPos;
	};

	struct tTrace
	{
		tParams mParams;
		cCircularBuffer<tVector, Eigen::aligned_allocator<tVector>> mPosTraj;
		cCircularBuffer<tEndEffectorPos, Eigen::aligned_allocator<tEndEffectorPos>> mEndEffPos;
		std::vector<bool> mEndContact;
	};

	int mBufferSize;
	double mSamplePeriod;
	double mTimer;
	std::vector<tTrace, Eigen::aligned_allocator<tTrace>> mTraces;

	virtual void ResetTimer();
	virtual void BuildTrace(const tParams& params, tTrace& out_trace) const;
	virtual void ResetTrace(tTrace& out_trace) const;
	virtual void UpdateTrace(tTrace& out_trace) const;
	virtual void UpdateTraceTraj(tTrace& out_trace) const;
	virtual void UpdateTraceEndPos(tTrace& out_trace) const;

	virtual const tTrace& GetTrace(int handle) const;
	virtual tTrace& GetTrace(int handle);
	virtual tVector CalcTracePos(const tTrace& trace) const;

	virtual void DrawTrace(const tTrace& trace) const;
	virtual void DrawTraceTraj(const tTrace& trace) const;
	virtual void DrawTraceEndPos(const tTrace& trace) const;

	virtual void SetTraceColIdx(int idx, tTrace& out_trace) const;
};
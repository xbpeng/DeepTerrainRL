#pragma once

#include "sim/Ground.h"
#include "sim/TerrainGen2D.h"
#include "util/Rand.h"

class cGroundVar2D : public cGround
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum ePadding
	{
		ePaddingNone,
		ePaddingFlat,
		ePaddingMax
	};

	struct tParams
	{
		tParams();
		double mFriction;
		double mSegmentWidth;
		ePadding mPadding;
	};

	cGroundVar2D();
	virtual ~cGroundVar2D();

	virtual void Init(std::shared_ptr<cWorld> world, const tParams& params,
					const tVector& bound_min, const tVector& bound_max);
	virtual void Update(const tVector& bound_min, const tVector& bound_max);
	virtual void Clear();

	virtual double SampleHeight(const tVector& pos) const;
	virtual double SampleHeight(const tVector& pos, bool& out_valid_sample) const;
	virtual void SampleHeight(const Eigen::MatrixXd& pos, Eigen::VectorXd& out_h) const;

	virtual eGroundType GetGroundType() const;

	virtual int GetGridWidth() const;
	virtual int GetGridLength() const;
	virtual tVector GetVertex(int i, int j) const;
	virtual tVector CalcGridCoord(const tVector& pos) const;
	virtual tVector GetPos() const;
	virtual double GetWidth() const;

	virtual void SetTerrainFunc(cTerrainGen2D::tTerrainFunc func);
	virtual void SeedRand(unsigned long seed);

protected:
	static const int gNumSegments = 2;
	enum eAlignMode
	{
		eAlignMin,
		eAlignMax
	};
	
	struct tSegment : public cSimObj
	{
		static const int gGridLength;
		static const double gGridSpacingX;
		static const double gGridSpacingZ;

		tSegment();
		virtual ~tSegment();
		
		void Init(std::shared_ptr<cWorld> world, double min_x, double friction);
		void Clear();
		bool IsEmpty() const;
		const double GetMinX() const;
		const double GetMaxX() const;

		int GetGridWidth() const;
		int GetGridLength() const;
		double GetWidth() const;
		double GetLength() const;

		tVector GetScaling() const;

		tVector GetVertex(int i, int j) const;
		int CalcDataIdx(int i, int j) const;
		double SampleHeight(const tVector& pos, bool& out_valid_sample) const;
		double GetStartHeight() const;
		double GetEndHeight() const;
		
		tVector CalcGridCoord(const tVector& pos) const;
		bool OutsideGrid(const tVector& coord) const;
		tVector ClampCoord(const tVector& coord) const;

		std::vector<float> mData;
		double mMinX;
	};

	tParams mParams;
	cRand mRand;
	cTerrainGen2D::tTerrainFunc mTerrainFunc;

	bool mFlipSeg;
	std::unique_ptr<tSegment> mSegments[gNumSegments];

	virtual void ResetParams();

	virtual bool OutsideGrid(const tVector& coord) const;

	virtual void InitSegments(const tVector& bound_min, const tVector& bound_max);
	virtual void ClearSegments();
	virtual int GetSegID(int s) const;
	virtual const std::unique_ptr<tSegment>& GetSegment(int s) const;
	virtual std::unique_ptr<tSegment>& GetSegment(int s);
	virtual const std::unique_ptr<tSegment>& GetMinSegment() const;
	virtual const std::unique_ptr<tSegment>& GetMaxSegment() const;

	virtual eAlignMode GetSegAlignMode(int seg_id) const;
	virtual void BuildSegment(int seg_id, double bound_min, double bound_max, 
							eAlignMode align_mode, const tVector& fix_point);
	virtual void AddPadding(int seg_id, double bound_min, double bound_max);

	virtual double GetMinX() const;
	virtual double GetMidX() const;
	virtual double GetMaxX() const;
};

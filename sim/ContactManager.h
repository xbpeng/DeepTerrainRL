#pragma once

#include <memory>
#include "util/MathUtil.h"

class cWorld;

class cContactManager
{
public:
	const static int gInvalidID;
	const static short gFlagAll = -1;
	const static short gFlagNone = 0;
	const static short gFlagRayTest = 1;

	struct tContactHandle
	{
		int mID;
		int mFlags;
		int mFilterFlags;

		tContactHandle();
		bool IsValid() const;
	};

	cContactManager(cWorld& world);
	virtual ~cContactManager();

	virtual void Init();
	virtual void Reset();
	virtual void Clear();
	virtual void Update();

	virtual tContactHandle RegisterContact(int contact_flags, int filter_flags);
	virtual void UpdateContact(const cContactManager::tContactHandle& handle);
	virtual int GetNumEntries() const;
	virtual bool IsInContact(const tContactHandle& handle) const;
	virtual tVector GetContactPt(const tContactHandle& handle) const;

protected:
	struct tContactEntry
	{
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		tContactEntry();

		int mFlags;
		int mFilterFlags;
		bool mInContact;
		tVector mContactPt;
	};

	cWorld& mWorld;
	std::vector<tContactEntry, Eigen::aligned_allocator<tContactEntry>> mContactEntries;

	virtual int RegisterNewID();
	virtual void ClearContacts();
	virtual bool IsValidContact(const tContactHandle& h0, const tContactHandle& h1) const;
};
#include "ContactManager.h"
#include "sim/World.h"
#include "SimObj.h"
#include <iostream>

const int cContactManager::gInvalidID = -1;

cContactManager::tContactHandle::tContactHandle()
{
	mID = gInvalidID;
	mFlags = gFlagAll;
	mFilterFlags = gFlagAll;
}

cContactManager::tContactEntry::tContactEntry()
{
	mFlags = gFlagAll;
	mFilterFlags = gFlagAll;
	mInContact = false;
}

bool cContactManager::tContactHandle::IsValid() const
{
	return mID != gInvalidID;
}

cContactManager::cContactManager(cWorld& world)
	: mWorld(world)
{

}

cContactManager::~cContactManager()
{

}

void cContactManager::Init()
{
	Clear();
}

void cContactManager::Reset()
{
	for (int i = 0; i < GetNumEntries(); ++i)
	{
		tContactEntry& entry = mContactEntries[i];
		entry.mInContact = false;
	}
}

void cContactManager::Clear()
{
	mContactEntries.clear();
}

void cContactManager::Update()
{
	ClearContacts();
	std::unique_ptr<btDiscreteDynamicsWorld>& bt_world = mWorld.GetInternalWorld();

	int num_manifolds = bt_world->getDispatcher()->getNumManifolds();
	for (int i = 0; i < num_manifolds; ++i)
	{
		btPersistentManifold* mani = bt_world->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* obj0 = static_cast<const btCollisionObject*>(mani->getBody0());
		const btCollisionObject* obj1 = static_cast<const btCollisionObject*>(mani->getBody1());

		int num_contacts = mani->getNumContacts();
		for (int j = 0; j < num_contacts; ++j)
		{
			btManifoldPoint& pt = mani->getContactPoint(j);
			// std::cout << " Detected contact1: " << pt.getDistance() << std::endl;
			btScalar dist_tol = 0.001f;
			if (pt.getDistance() <= dist_tol)
			{
				// std::cout << " Detected contact: " << std::endl;
				const cSimObj* sim_obj0 = static_cast<const cSimObj*>(obj0->getUserPointer());
				const cSimObj* sim_obj1 = static_cast<const cSimObj*>(obj1->getUserPointer());

				const tContactHandle& h0 = sim_obj0->GetContactHandle();
				const tContactHandle& h1 = sim_obj1->GetContactHandle();
			
				bool valid_contact = IsValidContact(h0, h1);
				if (valid_contact)
				{
					if (h0.IsValid())
					{
						mContactEntries[h0.mID].mInContact = true;
						mContactEntries[h0.mID].mContactPt = mWorld.GetManifoldPtA(pt);
					}

					if (h1.IsValid())
					{
						mContactEntries[h1.mID].mInContact = true;
						mContactEntries[h0.mID].mContactPt = mWorld.GetManifoldPtB(pt);
					}
				}
			}
		}
	}
}

cContactManager::tContactHandle cContactManager::RegisterContact(int contact_flags, int filter_flags)
{
	tContactHandle handle;
	handle.mFlags = contact_flags;
	handle.mFilterFlags = filter_flags;
	handle.mID = RegisterNewID();

	tContactEntry& entry = mContactEntries[handle.mID];
	entry.mFlags = contact_flags;
	entry.mFilterFlags = filter_flags;

	assert(handle.IsValid());
	return handle;
}

void cContactManager::UpdateContact(const cContactManager::tContactHandle& handle)
{
	assert(handle.IsValid());
	tContactEntry& entry = mContactEntries[handle.mID];
	entry.mFlags = handle.mFlags;
	entry.mFilterFlags = handle.mFilterFlags;
}

int cContactManager::GetNumEntries() const
{
	return static_cast<int>(mContactEntries.size());
}

bool cContactManager::IsInContact(const tContactHandle& handle) const
{
	if (handle.IsValid())
	{
		return mContactEntries[handle.mID].mInContact;
	}
	return false;
}

tVector cContactManager::GetContactPt(const tContactHandle& handle) const
{
	if (handle.IsValid())
	{
		return mContactEntries[handle.mID].mContactPt;
	}
	return tVector::Zero();
}

int cContactManager::RegisterNewID()
{
	int id = gInvalidID;
	id = static_cast<int>(mContactEntries.size());
	mContactEntries.resize(id + 1);
	return id;
}

void cContactManager::ClearContacts()
{
	int num_entries = GetNumEntries();
	for (int i = 0; i < num_entries; ++i)
	{
		tContactEntry& curr_entry = mContactEntries[i];
		curr_entry.mInContact = false;
		curr_entry.mContactPt.setZero();
	}
}

bool cContactManager::IsValidContact(const tContactHandle& h0, const tContactHandle& h1) const
{
	bool valid_h0 = ((h0.mFilterFlags & h1.mFlags) != 0);
	bool valid_h1 = ((h1.mFilterFlags & h0.mFlags) != 0);
	bool valid_contact = valid_h0 && valid_h1;
	return valid_contact;
}

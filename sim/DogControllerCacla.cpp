#include "DogControllerCacla.h"

cDogControllerCacla::cDogControllerCacla() : cTerrainRLCharController(),
											cDogController(),
											cBaseControllerCacla()
{
	mExpBaseActionRate = 0.2;
	mExpNoise = 0.2;
}

cDogControllerCacla::~cDogControllerCacla()
{
}

bool cDogControllerCacla::IsCurrActionCyclic() const
{
	return false;
}

void cDogControllerCacla::UpdateAction()
{
	cDogController::UpdateAction();
#if defined(ENABLE_DEBUG_VISUALIZATION)
	RecordVal();
#endif // ENABLE_DEBUG_VISUALIZATION
}
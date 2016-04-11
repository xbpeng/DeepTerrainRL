#include "RaptorControllerCacla.h"

cRaptorControllerCacla::cRaptorControllerCacla() : cTerrainRLCharController(),
											cRaptorController(),
											cBaseControllerCacla()
{
	mExpBaseActionRate = 0.2;
	mExpNoise = 0.15;
}

cRaptorControllerCacla::~cRaptorControllerCacla()
{
}

void cRaptorControllerCacla::UpdateAction()
{
	cRaptorController::UpdateAction();
#if defined(ENABLE_DEBUG_VISUALIZATION)
	RecordVal();
#endif // ENABLE_DEBUG_VISUALIZATION
}

bool cRaptorControllerCacla::IsCurrActionCyclic() const
{
	return false;
}
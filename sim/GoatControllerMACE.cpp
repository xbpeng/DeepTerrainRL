#include "GoatControllerMACE.h"

cGoatControllerMACE::cGoatControllerMACE()
{
}

cGoatControllerMACE::~cGoatControllerMACE()
{
}

tVector cGoatControllerMACE::GetTargetVel() const
{
	return tVector(2, 0, 0, 0);
}
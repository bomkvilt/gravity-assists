#include "planetBase.hpp"


PlanetBase::PlanetBase(IPlanetEphemerides::ptr ephemerides)
	: ephemerides(ephemerides)
{
	SetTime(0);
	SetGravParam(ephemerides->GetGM());
}

void PlanetBase::SetTime(float newTime) 
{
	time = newTime;
	auto newLocation = ephemerides->GetLocation(time);
	SetLocation(newLocation);
}

float PlanetBase::GetTime() const 
{ 
	return time;
}

float PlanetBase::GetPeriod() const 
{ 
	return ephemerides->GetT();
}

#ifndef PATHFINDER__PLANETBASE_HPP
#define PATHFINDER__PLANETBASE_HPP

#include "body.hpp"
#include "planets.hpp"
#include "interfaces/iPlanetEphemerides.hpp"



class PlanetBase : public Body
{
public:
	PlanetBase(IPlanetEphemerides::ptr ephemerides);
	
	void SetTime(float newTime);
	
	float GetTime() const;

	float GetPeriod() const;

private:
	IPlanetEphemerides::ptr ephemerides;
	
	float time = 0;
};


#endif //!PATHFINDER__PLANETBASE_HPP

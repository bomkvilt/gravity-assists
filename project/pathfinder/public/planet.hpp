#ifndef PATHFINDER__PLANET_HPP
#define PATHFINDER__PLANET_HPP

#include "body.hpp"
#include "interfaces/IPlanetEphemerics.hpp"


enum class EPlanet {
	  eMercury
	, eVenus
	, eEarth
	, eMoon
	, eMars
	, eJupter
};


struct IPlanet : public Body
{
public:
	virtual IPlanet* OnTime (float time) = 0;
	virtual void	 SetTime(float time) = 0;
	virtual float	 GetTime() const = 0;

	virtual float GetPeriod() const = 0;
};


class Planet : public IPlanet
{
public:
	Planet(EPlanet planet) {};

	IPlanet* OnTime (float time) override { return nullptr; }
	void	 SetTime(float time) override {};
	float	 GetTime() const override { return 0; };

	float GetPeriod() const override { return 0; };

private:
	IPlanetEphemerics* ephemerics;
};


#endif //!PATHFINDER__PLANET_HPP

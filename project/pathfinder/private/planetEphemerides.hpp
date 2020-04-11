#ifndef PATHFINDER__PLANETEPHEMERIDES_HPP
#define PATHFINDER__PLANETEPHEMERIDES_HPP

#include "interfaces/iPlanetEphemerides.hpp"
#include "planets.hpp"



class PlanetEphemerides : public IPlanetEphemerides
{
public:

	PlanetEphemerides(EPlanet planetName, const std::string& absTime);

	FVector GetLocation(float time) override;

	float GetGM() const override;

	float GetT() const override;

protected:

	std::string name;
	std::string primaryBody;
	double timeRoot = 0;
	double T  = 0;
	double GM = 0;
};


#endif //!PATHFINDER__PLANETEPHEMERIDES_HPP

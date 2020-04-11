#include "planet.hpp"
#include "planetEphemerides.hpp"



Planet::Planet(EPlanet planet, const std::string& initialTime)
	: PlanetBase(std::make_shared<PlanetEphemerides>(planet, initialTime))
{}

#ifndef PATHFINDER__PLANET_HPP
#define PATHFINDER__PLANET_HPP

#include "planetBase.hpp"
#include "planets.hpp"



class Planet : public PlanetBase
{
public:
	Planet(EPlanet planet, const std::string& initialTime);
	
};


#endif //!PATHFINDER__PLANET_HPP

#include "getPlanetName.hpp"
#include  <boost/algorithm/string.hpp>



namespace utiles 
{
	using Pathfinder::PlanetScript::EPlanet;

	EPlanet GetPlanetName(const std::string& name_)
	{
		auto name = boost::to_lower_copy(name_);
		if (name == "mercury") return EPlanet::eMercury;
		if (name == "venus"  ) return EPlanet::eVenus;
		if (name == "earth"  ) return EPlanet::eEarth;
		if (name == "mars"   ) return EPlanet::eMars;
		if (name == "jupter" ) return EPlanet::eJupter;
		if (name == "moon"   ) return EPlanet::eMoon;
		throw std::runtime_error("unexpected planet name: " + name);
	}
}

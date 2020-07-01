#ifndef MAIN__TRACEPLANET_HPP
#define MAIN__TRACEPLANET_HPP

#include "utiles/getPlanetName.hpp"
#include <fstream>



int TracePlanet(
	const std::string& outFile, 
	const std::string& planetName, 
	const std::string& startDate, 
	FReal bgn,
	FReal end,
	FReal step
) {
	using namespace Pathfinder::PlanetScript;
	auto ep = PlanetScript(utiles::GetPlanetName(planetName), startDate);
	auto os = std::ofstream(outFile);
	if (!os)
	{
		throw std::runtime_error("Passed path cannot be opend on write");
	}
	
	auto points = std::vector<FVector>();
	for (auto t = bgn; t < end; t += step)
	{
		points.push_back(ep.GetLocation(t));
	}

	auto ar = reflect::Archiver();
	ar.AddField(points, "points");
	auto data = ar.Save();
	os << data;

	return 0;
}


#endif //!MAIN__TRACEPLANET_HPP

#ifndef MAIN__TRACETRAJECTORY_HPP
#define MAIN__TRACETRAJECTORY_HPP

#include "pathfinder.hpp"
#include <filesystem>
#include <iostream>
#include <fstream>



int TraceTrajectory(std::string outPath, std::string path, FReal fraction)
{
	auto is = std::ifstream(path);
	if (!is)
	{
		throw std::runtime_error("Passed trajectory path cannot be opend");
	}

	auto os = std::ofstream(outPath);
	if (!os)
	{
		throw std::runtime_error("Passed output path cannot be opend");
	}

	auto chain = Pathfinder::PathFinder::FlightChain();
	{
		auto ar = reflect::Archiver();
		ar.Load(std::string(std::istreambuf_iterator(is), {}));
		
		chain.Unmarshal(ar);
	}

	auto points = std::vector<FVector>();
	for (const auto& link : chain.chain)
	{
		auto min = link.link.q0;
		auto max = link.link.q1;
		auto step = (max - min) * fraction;

		for (auto q = min; q <= max; q += step)
		{
			auto V = link.link.GetTragectoryPoint(q);
			points.push_back(V);
		}
	}
	
	auto ar = reflect::Archiver();
	ar.AddField(points, "points");
	auto data = ar.Save();
	os << data;

	return 0;
}



#endif //!MAIN__TRACETRAJECTORY_HPP

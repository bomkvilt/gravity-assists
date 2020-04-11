#ifndef PATHFINDER__FIRSTAPPROX_HPP
#define PATHFINDER__FIRSTAPPROX_HPP

#include "firstApproxUtiles.hpp"



class FirstApprox
{
public:
	PlanetBase A;	// planet to departure from
	PlanetBase B;	// planet to arrival to
	PlanetBase C;	// baricenter
	float t0;	// abs departure time, s
	float te;	// abs max arrival time, s
	float ts;	// 
	
public:
	// \note: C must be placed on {0, 0, 0}
	// \todo: add support of non-zero BCs
	FirstApprox(PlanetBase& A, PlanetBase& B, PlanetBase& C, float t0, int n = 30);

	// 
	auto FindTrajectories(const std::vector<float>& df0s)->std::vector<FirstApproxUtiles::TragectoryFinder>;
};


#endif //!PATHFINDER__FIRSTAPPROX_HPP

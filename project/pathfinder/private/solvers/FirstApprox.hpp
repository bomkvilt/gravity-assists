#ifndef PATHFINDER__FIRSTAPPROX_HPP
#define PATHFINDER__FIRSTAPPROX_HPP

#include "pathfinder.hpp"



namespace Pathfinder::Solvers
{
	auto FirstApprox(const Mission& mission, FReal t0)->std::vector<PathFinder::FlightChain>;
}


#endif //!PATHFINDER__FIRSTAPPROX_HPP

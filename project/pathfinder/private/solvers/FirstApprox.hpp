#ifndef PATHFINDER__SOLVER_FIRSTAPPROX_HPP
#define PATHFINDER__SOLVER_FIRSTAPPROX_HPP

#include "pathfinder.hpp"



namespace Pathfinder::Solvers
{
	auto FirstApprox(Mission& mission, FReal t0)->std::vector<PathFinder::FlightChain>;
}


#endif //!PATHFINDER__SOLVER_FIRSTAPPROX_HPP

#ifndef PATHFINDER__SOLVER_SECONDAPPROX_HPP
#define PATHFINDER__SOLVER_SECONDAPPROX_HPP

#include "pathfinder.hpp"



namespace Pathfinder::Solvers
{
	auto SecondApprox(
		  Mission& mission
		, const PathFinder::FlightChain& flight
		, PathFinder::Functionality functionality
	)->std::tuple<PathFinder::FlightChain, FReal>;
}


#endif //!PATHFINDER__SOLVER_SECONDAPPROX_HPP

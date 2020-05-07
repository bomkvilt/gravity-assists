#ifndef PATHFINDER__SECONDAPPROX_HPP
#define PATHFINDER__SECONDAPPROX_HPP

#include "pathfinder.hpp"



namespace Pathfinder::Solvers
{
	std::tuple<PathFinder::FlightChain, FReal> SecondApprox(
		  const Mission& mission
		, const PathFinder::FlightChain& flight
		, const PathFinder::Functionality& functionality
		// , FReal tMin
		// , FReal tMax
	);
}


#endif //!PATHFINDER__SECONDAPPROX_HPP

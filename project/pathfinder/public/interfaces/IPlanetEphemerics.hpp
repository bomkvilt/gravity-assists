#ifndef PATHFINDER__IPLANETEPHEMERICS_HPP
#define PATHFINDER__IPLANETEPHEMERICS_HPP

#include "math/math.hpp"



// Interface of an entity scripts planets' motion
struct IPlanetEphemerics
{
	// returns (position, velocity) pair the body have at the time
	// \note: time - time since J2000
	virtual auto GetKinematics(float time)->std::tuple<FVector, FVector> = 0;

	virtual ~IPlanetEphemerics() = default;
};


#endif //!PATHFINDER__IPLANETEPHEMERICS_HPP

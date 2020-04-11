#ifndef PATHFINDER__IPLANETEPHEMERIDES_HPP
#define PATHFINDER__IPLANETEPHEMERIDES_HPP

#include "math/math.hpp"



// stateless connection to ephemerides source
struct IPlanetEphemerides : boost::noncopyable
{
	using ptr = std::shared_ptr<IPlanetEphemerides>;

	virtual ~IPlanetEphemerides() = default;

	virtual FVector GetLocation(float time) = 0;

	virtual float GetGM() const = 0;

	virtual float GetT() const = 0;
};


#endif //!PATHFINDER__IPLANETEPHEMERIDES_HPP

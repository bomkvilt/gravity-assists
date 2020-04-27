#ifndef PATHFINDER__EPHEMERIDES_HPP
#define PATHFINDER__EPHEMERIDES_HPP

#include "math/math.hpp"



namespace Pathfinder::Ephemerides
{
	// IEphemerides is a connaction to a planet ephemerides engine/database
	struct IEphemerides
	{
		using ptr = std::shared_ptr<IEphemerides>;

		virtual ~IEphemerides() = default;

		virtual float GetT (float time) const = 0;
		virtual float GetGM(float time) const = 0;
		virtual FVector GetLocation(float time) const = 0;
		virtual FVector GetVelocity(float time) const = 0;
		virtual auto GetMovement(float time)->std::tuple<FVector, FVector> const = 0;
	};
}


#endif //!PATHFINDER__EPHEMERIDES_HPP

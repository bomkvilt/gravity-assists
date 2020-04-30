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

		virtual FReal GetT (FReal time) const = 0;
		virtual FReal GetGM(FReal time) const = 0;
		virtual FVector GetLocation(FReal time) const = 0;
		virtual FVector GetVelocity(FReal time) const = 0;
		virtual auto GetMovement(FReal time)->std::tuple<FVector, FVector> const = 0;
	};
}


#endif //!PATHFINDER__EPHEMERIDES_HPP

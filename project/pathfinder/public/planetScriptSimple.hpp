#ifndef PATHFINDER__PLANETSCRIPTSIMPLE_HPP
#define PATHFINDER__PLANETSCRIPTSIMPLE_HPP

#include "interfaces/ephemerides.hpp"



namespace Pathfinder::PlanetScript
{
	struct PlanetScriptSimple : public Ephemerides::IEphemerides
	{
		FReal phase0 = 0; // [rad]
		FReal period = 0; // [s]
		FReal radius = 0; // [m]
		FReal GM = 0;     // [m3/s2]

		PlanetScriptSimple(FReal GM, FReal radius, FReal period, FReal phase);

		FVector GetLocation(FReal time) const override;
		FVector GetVelocity(FReal time) const override;
		auto    GetMovement(FReal time)->std::tuple<FVector, FVector> const override;

		FReal GetGM(FReal) const override;
		FReal GetT (FReal) const override;

		FReal GetPhase(FReal time) const;
		FReal GetOmega(FReal time) const;
	};
}


#endif //!PATHFINDER__PLANETSCRIPTSIMPLE_HPP

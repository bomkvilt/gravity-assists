#ifndef PATHFINDER__PLANETSCRIPTSIMPLE_HPP
#define PATHFINDER__PLANETSCRIPTSIMPLE_HPP

#include "interfaces/ephemerides.hpp"



namespace Pathfinder::PlanetScript
{
	struct PlanetScriptSimple : public Ephemerides::IEphemerides
	{
		float phase0 = 0; // [rad]
		float period = 0; // [s]
		float radius = 0; // [m]
		float GM = 0;     // [m3/s2]

		PlanetScriptSimple(float GM, float radius, float period, float phase);

		FVector GetLocation(float time) const override;
		FVector GetVelocity(float time) const override;
		auto    GetMovement(float time)->std::tuple<FVector, FVector> const override;

		float GetGM(float) const override;
		float GetT (float) const override;

		float GetPhase(float time) const;
		float GetOmega(float time) const;
	};
}


#endif //!PATHFINDER__PLANETSCRIPTSIMPLE_HPP

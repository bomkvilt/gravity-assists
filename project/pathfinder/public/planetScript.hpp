#ifndef PATHFINDER__PLANETSCRIPT_HPP
#define PATHFINDER__PLANETSCRIPT_HPP

#include "interfaces/ephemerides.hpp"


namespace Pathfinder::PlanetScript
{
	enum class EPlanet {
		  eNONE
		, eSun
		, eMercury
		, eVenus
		, eEarth
		, eMoon
		, eMars
		, eJupter
	};


	class PlanetScript : public Ephemerides::IEphemerides
	{
	public:
		PlanetScript(EPlanet planet, const std::string& J2000Time);

		float GetT (float time) const override;
		float GetGM(float time) const override;
		FVector GetLocation(float time) const override;
		FVector GetVelocity(float time) const override;
		auto GetMovement(float time)->std::tuple<FVector, FVector> const override;

	protected:
		std::string name;
		std::string center;
		double T  = 0;
		double t0 = 0;
		double GM = 0;
	};
}


#endif //!PATHFINDER__PLANETSCRIPT_HPP

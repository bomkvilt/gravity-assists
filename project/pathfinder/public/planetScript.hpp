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
	protected:
		using MovState = std::tuple<FVector, FVector>;
		using Chunk  = std::unordered_map<UInt64, MovState>;
		using Chunks = std::unordered_map<UInt64, Chunk>;

	public:
		PlanetScript(EPlanet planet, const std::string& J2000Time);

		FReal GetT (FReal time) const override;
		FReal GetGM(FReal time) const override;
		FVector GetLocation(FReal time) const override;
		FVector GetVelocity(FReal time) const override;
		auto GetMovement(FReal time)->std::tuple<FVector, FVector> const override;

		void MakeDiscret(FReal stepSize, FReal chunkSize);

		bool IsDiscret() const;

	protected:

		const MovState& GetMovement_D(FReal time) const;

		FVector GetLocation_C(FReal time) const;
		FVector GetVelocity_C(FReal time) const;
		auto GetMovement_C(FReal time)->std::tuple<FVector, FVector> const;

		void AddChunk(UInt64 chunkN);

	protected:
		std::string name;
		std::string center;
		FReal T  = 0;
		FReal t0 = 0;
		FReal GM = 0;

		FReal stepSize = 0;
		FReal chunkSize = 0;
		Chunks chunks;
	};
}


#endif //!PATHFINDER__PLANETSCRIPT_HPP

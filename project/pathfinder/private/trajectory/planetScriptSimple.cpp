#include "planetScriptSimple.hpp"



namespace Pathfinder::PlanetScript
{
	PlanetScriptSimple::PlanetScriptSimple(FReal GM, FReal radius, FReal period, FReal phase)
		: period(period)
		, radius(radius)
		, phase0(phase)
		, GM(GM)
	{}

	FVector PlanetScriptSimple::GetLocation(FReal time) const
	{
		auto newPhase = GetPhase(time);
		auto rotation = FQuat({ 0, 0, 1 }, RAD2DEG(newPhase));
		auto location = FVector(radius, 0, 0);
		return rotation * location;
	}

	FVector PlanetScriptSimple::GetVelocity(FReal time) const
	{
		using namespace Math;
		const auto v = radius * 2*Pi/period;
		const auto q = GetPhase(time);
		return {-v * Sin(q), v * Cos(q), 0};
	}

	auto PlanetScriptSimple::GetMovement(FReal time)->std::tuple<FVector, FVector> const
	{
		return { GetLocation(time), GetVelocity(time) };
	}

	FReal PlanetScriptSimple::GetGM(FReal) const
	{
		return GM;
	}

	FReal PlanetScriptSimple::GetT(FReal) const
	{
		return period;
	}

	FReal PlanetScriptSimple::GetPhase(FReal time) const
	{
		return phase0 + time * GetOmega(time);
	}
	
	FReal PlanetScriptSimple::GetOmega(FReal time) const
	{
		using namespace Math;
		return 2*Pi/period;
	}
}

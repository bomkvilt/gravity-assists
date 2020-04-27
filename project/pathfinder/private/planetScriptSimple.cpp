#include "planetScriptSimple.hpp"


namespace Pathfinder::PlanetScript
{
	PlanetScriptSimple::PlanetScriptSimple(float GM, float radius, float period, float phase)
		: period(period)
		, radius(radius)
		, phase0(phase)
		, GM(GM)
	{}

	FVector PlanetScriptSimple::GetLocation(float time) const
	{
		auto newPhase = GetPhase(time);
		auto rotation = FQuat({ 0, 0, 1 }, RAD2DEG(newPhase));
		auto location = FVector(radius, 0, 0);
		return rotation * location;
	}

	FVector PlanetScriptSimple::GetVelocity(float time) const
	{
		using namespace Math;
		const auto v = radius * 2*Pi/period;
		const auto q = GetPhase(time);
		return {v * Cos(q), -v * Sin(q), 0};
	}

	auto PlanetScriptSimple::GetMovement(float time)->std::tuple<FVector, FVector> const
	{
		return { GetLocation(time), GetVelocity(time) };
	}

	float PlanetScriptSimple::GetGM(float) const
	{
		return GM;
	}

	float PlanetScriptSimple::GetT(float) const
	{
		return period;
	}

	float PlanetScriptSimple::GetPhase(float time) const
	{
		return phase0 + time * GetOmega(time);
	}
	
	float PlanetScriptSimple::GetOmega(float time) const
	{
		using namespace Math;
		return 2*Pi/period;
	}
}

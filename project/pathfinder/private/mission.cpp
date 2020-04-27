#include "mission.hpp"
#include "keplerOrbit.hpp"


namespace Pathfinder
{
	bool NodeDeparture::Check(const FVector& Win, const FVector& Wout) const
	{
		return Wout.Size() > EscapeSpeed;
	}

	bool NodeArrival::Check(const FVector& Win, const FVector& Wout) const
	{
		using namespace Math;
		namespace kr = ::Pathfinder::Kepler;
		namespace el = ::Pathfinder::Kepler::Elliptic;
		const auto GM = Script->GetGM(0);
		const auto w0 = Win.Size();
		const auto w1 = kr::reorbit_v(w0, SphereRadius, ParkingRadius, GM);
		const auto w2 = el::v(0, 0, ParkingRadius, GM);
		if (ImpulseLimit_v > 0 && !Equal(w1, w2, ImpulseLimit_v))
		{
			return false;
		}

		return true;
	}
	
	bool NodeFlyBy::Check(const FVector& Win, const FVector& Wout) const
	{
		if (MissmatchLimit_v > 0 && !Math::Equal(Win.Size(), Wout.Size(), MissmatchLimit_v))
		{
			return false;
		}

		namespace kr = ::Pathfinder::Kepler;
		namespace hb = ::Pathfinder::Kepler::Hiperbolic;
		const auto GM = Script->GetGM(0);
		const auto w = Win.Size();
		const auto h = kr::h(w, SphereRadius, GM);
		if (h <= 0)
		{
			return false;
		}

		const auto bmin = hb::bmin(w, SphereRadius, PlanetRadius, GM);
		const auto dmax = hb::kink(w, bmin, SphereRadius, GM);
		const auto d = Math::Angle2(Win, Wout);
		if (d > dmax)
		{
			return false;
		}

		return true;
	}
}

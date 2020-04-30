#include "mission.hpp"
#include "keplerOrbit.hpp"


namespace Pathfinder::Utiles
{
	FReal GetEscapeImpulse(FReal r0, FReal r1, FReal vd, FReal GM)
	{
		namespace kr = ::Pathfinder::Kepler;
		using namespace Math;
		auto ve = kr::v(0, r1, GM);
		return vd >= ve
			? Sqrt(vd*vd - 2*GM/r1 + 2*GM/r0) - Sqrt(GM/r0)
			: Sqrt(GM/r0)*(Sqrt(2.) - 1) + ve - vd
			;
	}

	FReal GetParkingImpulse(FReal r0, FReal r1, FReal va, FReal GM)
	{
		namespace kr = ::Pathfinder::Kepler;
		// SC speed: v1<->r0 -> ?<->r1
		auto h0 = kr::h(va, r0, GM);
		auto vt = kr::v(h0, r1, GM);
		// impulse to correct orbit
		auto v1 = kr::Elliptic::v(0, 0, r1, GM);
		return Math::Abs(v1 - vt);
	}
}

namespace Pathfinder
{
	auto NodeDeparture::Check(const InNodeParams& in)->std::tuple<OutNodeParams, bool> const
	{
		// impulse = parkinkg(r=parking) -> escape(r1=sphere, h=0) -> departure(v=w1)
		auto params = OutNodeParams();
		params.Impulse = Utiles::GetEscapeImpulse(
			  ParkingRadius
			, SphereRadius
			, in.W1.Size()
			, Script->GetGM(0)
		);
		return { params, ImpulseLimit > 0 ? (params.Impulse < ImpulseLimit) : true };
	}
	
	auto NodeArrival::Check(const InNodeParams& in)->std::tuple<OutNodeParams, bool> const
	{
		// impulse = arrival(v=w0) -> transfer(v0=w0, r0=shere, r1=parking) -> parking(r=parking)
		auto params = OutNodeParams();
		params.Impulse = Utiles::GetParkingImpulse(
			  SphereRadius
			, ParkingRadius
			, in.W0.Size()
			, Script->GetGM(0)
		);
		return { params, ImpulseLimit > 0 ? (params.Impulse < ImpulseLimit) : true };
	}
	
	auto NodeFlyBy::Check(const InNodeParams& in)->std::tuple<OutNodeParams, bool> const
	{
		auto w0 = in.W0.Size();
		auto w1 = in.W1.Size();
		auto params = OutNodeParams();

		// is the mismatch suitable?
		params.Mismatch = Math::Abs(w1 - w0);
		if (MismatchLimit > 0 && params.Mismatch > MismatchLimit)
		{
			return { params, false };
		}

		// is the orbit hiperbolic?
		namespace kr = ::Pathfinder::Kepler;
		auto GM = Script->GetGM(0);
		auto w = Math::Avg(w0, w1);
		auto h = kr::h(w, SphereRadius, GM);
		if (h <= 0)
		{
			return { params, false };
		}

		// can the kink be realised?
		const auto bmin = kr::Hiperbolic::bmin(w, SphereRadius, PlanetRadius, GM);
		const auto dmax = kr::Hiperbolic::kink(w, bmin, SphereRadius, GM);
		const auto d    = Math::Angle2(in.W0, in.W1);
		return { params, d < dmax };
	}
}

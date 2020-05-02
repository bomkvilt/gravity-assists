#include "nodes.hpp"
#include "keplerOrbit.hpp"


namespace Pathfinder::Utiles
{
	FReal GetEscapeImpulse(FReal r0, FReal r1, FReal h0, FReal v2, FReal GM)
	{
		namespace kr = ::Pathfinder::Kepler;
		auto v0 = kr::v(h0, r0, GM);
		auto ve = kr::v(0 , r1, GM);
		if (v2 > ve)
		{
			auto v1 = kr::v(v2, r1, r0, GM);
			return Math::Abs(v1 - v0);
		}
		else
		{
			auto v1 = kr::v(0, r0, GM);
			auto i1 = Math::Abs(v1 - v0);
			auto i2 = ve - v2;
			return i1 + i2;
		}
	}

	FReal GetParkingImpulse(FReal r0, FReal r1, FReal h1, FReal va, FReal GM)
	{
		namespace kr = ::Pathfinder::Kepler;
		auto h0 = kr::h(va, r0, GM);
		auto vt = kr::v(h0, r1, GM);
		auto v1 = kr::v(h1, r1, GM);
		return Math::Abs(v1 - vt);
	}

	FReal MakeCorrection(FReal var, FReal lim, FReal a, FReal k)
	{
		return a * std::pow(Math::Abs(lim - var), -k);
	}
}

namespace Pathfinder
{
	auto NodeDepartureBase::Check(const InNodeParams& in, bool bGenCorrection)->std::tuple<OutNodeParams, bool> const
	{
		// impulse = parkinkg(r=parking) -> escape(r1=sphere, h=0) -> departure(v=w1)
		auto params = OutNodeParams();
		params.Impulse = Utiles::GetEscapeImpulse(
			  ParkingRadius
			, SphereRadius
			, GetH0()
			, in.W1.Size()
			, Script->GetGM(0)
		);
		if (ImpulseLimit > 0 && params.Impulse > ImpulseLimit)
		{
			return { params, false };
		}
		if (ImpulseLimit > 0 && bGenCorrection)
		{
			params.Correction = Utiles::MakeCorrection(
				  params.Impulse
				, ImpulseLimit
				, A_impulse
				, K_impulse
			);
		}
		return { params, true };
	}
	
	auto NodeArrivalBase::Check(const InNodeParams& in, bool bGenCorrection)->std::tuple<OutNodeParams, bool> const
	{
		// impulse = arrival(v=w0) -> transfer(v0=w0, r0=shere, r1=parking) -> parking(r=parking)
		auto params = OutNodeParams();
		params.Impulse = Utiles::GetParkingImpulse(
			  SphereRadius
			, ParkingRadius
			, GetH1()
			, in.W0.Size()
			, Script->GetGM(0)
		);
		if (ImpulseLimit > 0 && params.Impulse > ImpulseLimit)
		{
			return { params, false };
		}
		if (ImpulseLimit > 0 && bGenCorrection)
		{
			params.Correction = Utiles::MakeCorrection(
				  params.Impulse
				, ImpulseLimit
				, A_impulse
				, K_impulse
			);
		}
		return { params, true };
	}
	
	auto NodeFlyBy::Check(const InNodeParams& in, bool bGenCorrection)->std::tuple<OutNodeParams, bool> const
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
		const auto d    = Math::Angle2(in.W0, in.W1, Math::EPosAngles());
		if (d >= dmax)
		{
			return { params, false };
		}
		
		if (bGenCorrection)
		{
			params.Correction += Utiles::MakeCorrection(
				  params.Mismatch
				, MismatchLimit
				, A_mismatch
				, K_mismatch
			);
			params.Correction += Utiles::MakeCorrection(
				  d
				, dmax
				, A_kink
				, K_kink
			);
		}
		return { params, true };
	}
}

namespace Pathfinder::NodeDeparture
{
	FReal EnergyDriven::GetH0() const
	{
		return h0;
	}

	FReal Circular::GetH0() const
	{
		auto GM = Script->GetGM(0);
		return -GM / ParkingRadius;
	}
}

namespace Pathfinder::NodeArrival
{
	FReal EnergyDriven::GetH1() const
	{
		return h1;
	}

	FReal Circular::GetH1() const
	{
		auto GM = Script->GetGM(0);
		return -GM / ParkingRadius;
	}
}

#include "trajectory/keplerOrbit.hpp"
#include "math/math.hpp"



namespace Pathfinder::Kepler
{
	FReal r(FReal p, FReal e, FReal q)
	{
		return p / (1 + e * Math::Cos(q));
	}
	
	FReal h(FReal vi, FReal ri, FReal GM)
	{
		return vi*vi - 2*GM/ri;
	}

	FReal v(FReal hi, FReal ri, FReal GM)
	{
		return Math::Sqrt(hi + 2*GM/ri);
	}

	FReal v(FReal v0, FReal r0, FReal r1, FReal GM)
	{
		// v0^2 - 2 GM/r0 == v1^2 - 2 GM/r1
		return Math::Sqrt(v0*v0 - 2*GM/r0 + 2*GM/r1);
	}
}


namespace Pathfinder::Kepler::Elliptic
{
	auto epwqq(FReal r0, FReal r1, FReal Q0, FReal Q1, FReal f0) -> std::tuple<_epwqq, bool>
	{
		if (Math::Equal(Q0, f0, 10e-5))
		{
			return { _epwqq(), false };
		}
		auto w0 = w(r0, r1, Q0, Q1, f0);
		for (auto k : { 0, 1 })
		{
			auto wk = w0 + k*Math::Pi;
			auto [q0, q1] = qq(Q0, Q1, wk);
			auto [e , p ] = ep(r0, r1, q0, q1);
			
			if (p <= 0 || Math::Abs(e) > 1)
			{
				break;
			}
			if (0 <= e && e < 0.99)
			{
				return { {e, p, wk, q0, q1}, true };
			}
		}
		return { _epwqq(), false };
	}

	auto ep(FReal r0, FReal r1, FReal q0, FReal q1) -> std::tuple<FReal, FReal>
	{
		using namespace Math;
		auto delta = r0/r1 - 1;
		if (!Equal(delta, 0, 10e-5))
		{
			auto C0 = Cos(q0);
			auto C1 = Cos(q1);
			auto den = r0 * C0 - r1 * C1;
			return {
				(r1 - r0) / den,
				(C0 - C1) / den * r0 * r1
			};
		}
		return { 0.f, r0 };
	}
	
	auto qq(FReal Q0, FReal Q1, FReal w) -> std::tuple<FReal, FReal>
	{
		auto q0 = NZ(Q0 + w);
		auto q1 = NZ(Q1 + w);
		if (q1 > q0)
		{
			return { q0, q1 };
		}
		return { q0, q1 + 2*Math::Pi };
	}
	
	FReal w(FReal r0, FReal r1, FReal Q0, FReal Q1, FReal f0)
	{
		using namespace Math;
		auto delta = r0/r1 - 1;
		if (!Equal(delta, 0, 10e-5))
		{
			auto num = 1 - Cos(Q1 - Q0);
			auto den = delta*Tan(Q0 - f0) - Sin(Q1 - Q0);
			assert(den != 0);
			return Atan(num / den) - Q0;
		}
		return Pi/2 * Sign(Q0 - Q1);
	}
	
	bool bf(FReal Q0, FReal f0)
	{
		return NZ(f0 - Q0) < Math::Pi;
	}
	
	FReal a(FReal e, FReal p)
	{
		return p / (1 - e*e);
	}

	FReal E(FReal qi, FReal ri, FReal e)
	{
		using namespace Math;
		if (Equal(e, 0, 10e-7))
		{
			return qi;
		}
		auto Cq = Cos(qi);
		auto AC = Acos((e + Cq) / (1 + e*Cq));
		return	qi < 1*Pi ? +AC + 0*Pi
			:	qi < 2*Pi ? -AC + 2*Pi
			:	qi < 3*Pi ? +AC + 2*Pi
			:				-AC + 4*Pi
			;
	}

	FReal M(FReal Ei, FReal e)
	{
		using namespace Math;
		return Ei - e * Sin(Ei);
	}

	FReal dt(FReal M0, FReal M1, FReal a, FReal GM, bool bf)
	{
		using namespace Math;
		auto c = Sqrt(a / GM * a * a);
		return bf
			? c * (0*Pi + (M1 - M0))
			: c * (2*Pi - (M1 - M0))
			;
	}

	FReal v(FReal qi, FReal e, FReal p, FReal GM)
	{
		using namespace Math;
		auto c = GM / p;
		return Sqrt(c * (1 + 2*e*Cos(qi) + e*e));
	}

	FReal f(FReal Qi, FReal qi, FReal e, bool bf)
	{
		using namespace Math;
		auto AT = Atan((e*Sin(qi)) / (1 + e*Cos(qi)));
		auto fi = Qi + Pi/2 - AT;
		return NZ(bf ? fi : fi + Pi);
	}

	FReal NZ(FReal f)
	{
		auto pi2 = 2 * Math::Pi;
		return f - pi2 * std::floor(f / pi2);
	}
}


namespace Pathfinder::Kepler::Hiperbolic
{
	FReal bmin(FReal v, FReal r, FReal rpl, FReal GM)
	{
		using namespace Math;
		const auto s1 = rpl / v;
		const auto s2 = 2*GM / rpl;
		const auto s3 = 2*GM / r;
		return s1 * Sqrt(s2 + v*v - s3);
	}
	
	FReal kink(FReal v, FReal b, FReal r, FReal GM)
	{
		using namespace Math;
		const auto v2 = v*v;
		const auto b2 = b*b;
		const auto v4 = v2*v2;
		const auto num = b2*v2 - r*GM;
		const auto den = r * (r*GM*GM + b2 * (r*v4 - 2*v2*GM));
		return 2 * Acos(num / den);
	}
}

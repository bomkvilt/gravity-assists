#include "keplerOrbit.hpp"
#include "math/math.hpp"


namespace Pathfinder::Kepler
{
	float h(float vi, float ri, float GM)
	{
		return vi*vi - 2*GM/ri;
	}

	float reorbit_v(float v0, float r0, float r1, float GM)
	{
		// v0^2 - 2 GM/r0 == v1^2 - 2 GM/r1
		return Math::Sqrt(v0*v0 - 2*GM/r0 + 2*GM/r1);
	}
}


namespace Pathfinder::Kepler::Elliptic
{
	auto epwqq(float r0, float r1, float Q0, float Q1, float f0) -> std::tuple<_epwqq, bool>
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

	auto ep(float r0, float r1, float q0, float q1) -> std::tuple<float, float>
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
	
	auto qq(float Q0, float Q1, float w) -> std::tuple<float, float>
	{
		auto q0 = NZ(Q0 + w);
		auto q1 = NZ(Q1 + w);
		if (q1 > q0)
		{
			return { q0, q1 };
		}
		return { q0, q1 + 2*Math::Pi };
	}
	
	float w(float r0, float r1, float Q0, float Q1, float f0)
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
	
	bool bf(float Q0, float f0)
	{
		return NZ(f0 - Q0) < Math::Pi;
	}
	
	float a(float e, float p)
	{
		return p / (1 - e*e);
	}

	float E(float qi, float ri, float e, float a)
	{
		using namespace Math;
		if (Equal(e, 0, 10e-7))
		{
			return qi;
		}
		auto AC = Acos((a - ri) / a / e);
		return	qi < 1*Pi ? +AC + 0*Pi
			:	qi < 2*Pi ? -AC + 2*Pi
			:	qi < 3*Pi ? +AC + 2*Pi
			:				-AC + 4*Pi
			;
	}

	float M(float Ei, float e)
	{
		using namespace Math;
		return Ei - e * Sin(Ei);
	}

	float dt(float M0, float M1, float a, float GM, bool bf)
	{
		using namespace Math;
		auto c = Sqrt(a* a / GM * a);
		return bf
			? c * (0*Pi + (M1 - M0))
			: c * (2*Pi - (M1 - M0))
			;
	}

	float v(float qi, float e, float p, float GM)
	{
		using namespace Math;
		auto c = GM / p;
		return Sqrt(c * (1 + 2*e*Cos(qi) + e*e));
	}

	float f(float Qi, float qi, float e, bool bf)
	{
		using namespace Math;
		auto AT = Atan((e*Sin(qi)) / (1 + e*Cos(qi)));
		auto fi = Qi + Pi/2 - AT;
		return NZ(bf ? fi : fi + Pi);
	}

	float NZ(float f)
	{
		auto pi2 = 2 * Math::Pi;
		return f - pi2 * std::floor(f / pi2);
	}
}


namespace Pathfinder::Kepler::Hiperbolic
{
	float bmin(float v, float r, float rpl, float GM)
	{
		using namespace Math;
		const auto s1 = rpl / v;
		const auto s2 = 2*GM / rpl;
		const auto s3 = 2*GM / r;
		return s1 * Sqrt(s2 + v*v - s3);
	}
	
	float kink(float v, float b, float r, float GM)
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

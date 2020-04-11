#include "firstApproxUtiles.hpp"


namespace FirstApproxUtiles::finders
{
	auto FindPEW(float Q0, float Q1, float r0, float r1, float f0) -> std::tuple<float, float, float, bool>
	{
		auto w = FindPeriapsisAngle(Q0, Q1, r0, r1, f0);
		if (auto [p, e, ok] = FindPE(Q0, Q1, r0, r1, w); ok)
		{
			return { p, e, w, true };
		}
		w += Math::Pi;
		if (auto [p, e, ok] = FindPE(Q0, Q1, r0, r1, w); ok)
		{
			return { p, e, w, true };
		}
		return { 0.f, 0.f, 0.f, false };
	}

	auto FindPE(float Q0, float Q1, float r0, float r1, float w)->std::tuple<float, float, bool>
	{
		auto e = FindEccentricity(Q0, Q1, r0, r1, w);
		if (e < 0 || e >= 1)
		{
			return { 0.f, 0.f, false };
		}
		auto p = FindParameter(Q0, Q1, r0, r1, w);
		if (p < 0)
		{
			return { 0.f, 0.f, false };
		}
		return { p, e, true };
	}

	float FindPeriapsisAngle(float Q0, float Q1, float r0, float r1, float f0)
	{
		using namespace Math;
		auto rFr = (r0 - r1) / r1;
		auto num = 1 - Cos(Q1 - Q0);
		auto den = rFr * Tan(Q0 - f0) - Sin(Q1 - Q0);
		return Atan(num / den) - Q0;
	}


	float FindParameter(float Q0, float Q1, float r0, float r1, float w)
	{
		using namespace Math;
		auto C0 = Cos(Q0 + w);
		auto C1 = Cos(Q1 + w);
		return r0 * r1 * (C0 - C1) / (C0 * r0 - C1 * r1);
	}

	float FindEccentricity(float Q0, float Q1, float r0, float r1, float w)
	{
		using namespace Math;
		auto C0 = Cos(Q0 + w);
		auto C1 = Cos(Q1 + w);
		return -(r0 - r1) / (C0 * r0 - C1 * r1);
	}

	float FindTime(float Q0, float Q1, float M, float p)
	{
		using namespace Math;
		return Sqrt(p * p * p / M) * (Q1 - Q0);
	}

	float FindVelocity(float Q, float w, float e, float p, float M)
	{
		using namespace Math;
		return Sqrt((M / p) * (e*e + 2*e*Cos(Q + w) + 1));
	}

	float FindVelAngle(float Q, float w, float e)
	{
		using namespace Math;
		auto num = e * Sin(Q + w);
		auto den = e * Cos(Q + w) + 1;
		return Q + Math::Pi/2 - Atan(num / den);
	}
}


namespace FirstApproxUtiles
{
	TragectoryFinder::TragectoryFinder(const FVector& R0, float Q0, float t0, float te, float ts, float M)
		: R0(R0), r0(R0.Size())
		, Q0(Q0)
		, t0(t0)
		, te(te)
		, ts(ts)
		, M(M)
	{}

	bool TragectoryFinder::FindTragectory(PlanetBase& B, float f0_)
	{
		using namespace finders;
		constexpr auto day = 3600 * 24;

		f0 = f0_;
		t1 = t0 + day;
		auto found = false;
		do
		{ // arrival planet's params
			B.SetTime(t1);
			R1 = B.GetLocation();
			r1 = R1.Size();
			Q1 = Q0 + Math::Angle2(R0, R1);

			// try to find elliptic orbit params
			if (auto [p_, e_, w_, ok] = FindPEW(Q0, Q1, r0, r1, f0); ok)
			{ // elliptical orbit exists
				p = p_;
				e = e_;
				w = w_;
			}
			else
			{ // no orbit exests, so let's try next time point
				t1 += ts;
				continue;
			}

			// time, passed for the fligth
			auto dt = FindTime(Q0, Q1, M, p);
			auto t2 = t0 + dt;
						
			if (Math::Abs(t2 - t1) < day)
			{ // solution was found
				found = true;
				v0 = FindVelocity(Q0, w, e, p, M);
				v1 = FindVelocity(Q1, w, e, p, M);
				f1 = FindVelAngle(Q1, w, e);
				t1 = t2;
				break;
			}
			else
			{ // next iteration
				t1 = Math::Avg(t2, t1);
			}
		} while (t1 < te);

		return found;
	}
}

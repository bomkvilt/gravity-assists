#include "gtest/gtest.h"
#include "firstApproxUtiles.hpp"
#include "boost/format.hpp"

namespace af = FirstApproxUtiles::finders;


struct firstApproxUtiles_tests : public testing::Test
{
	const float Q0 = DEG2RAD(10);
	const float Q1 = DEG2RAD(60);
	const float f0 = DEG2RAD(30);
	const float f1 = 3.87187;
	const float r0 = 1.496E+11;
	const float r1 = 2.244E+11;
	const float v0 = 35672.8;
	const float v1 = 26099.7;
	const float M =  1.327E+20;
	const float w = -0.680486;
	const float e = -0.951377;
	const float p =  2.51056E+10;
	const float t =  301347;

	void ASSERT_VALUES(float val_, float exp_)
	{
		auto val = (boost::format("%1.4d") % val_).str();
		auto exp = (boost::format("%1.4d") % exp_).str();
		ASSERT_EQ(exp, val);
	}
};


TEST_F(firstApproxUtiles_tests, find_w)
{
	auto res = af::FindPeriapsisAngle(Q0, Q1, r0, r1, f0);
	ASSERT_VALUES(res, w);
}

TEST_F(firstApproxUtiles_tests, find_e)
{
	{
		auto res = af::FindEccentricity(Q0, Q1, r0, r1, w);
		ASSERT_VALUES(res, +e);
	}
	{
		auto res = af::FindEccentricity(Q0, Q1, r0, r1, w + Math::Pi);
		ASSERT_VALUES(res, -e);
	}
}

TEST_F(firstApproxUtiles_tests, find_p)
{
	{
		auto res = af::FindParameter(Q0, Q1, r0, r1, w);
		ASSERT_VALUES(res, +p);
	}
	{
		auto res = af::FindParameter(Q0, Q1, r0, r1, w + Math::Pi);
		ASSERT_VALUES(res, +p);
	}
}

TEST_F(firstApproxUtiles_tests, find_v)
{
	{
		auto res = af::FindVelocity(Q0, w + Math::Pi, -e, +p, M);
		ASSERT_VALUES(res, v0);
	}
	{
		auto res = af::FindVelocity(Q1, w + Math::Pi, -e, +p, M);
		ASSERT_VALUES(res, v1);
	}
}

TEST_F(firstApproxUtiles_tests, find_f)
{
	{
		auto res = af::FindVelAngle(Q0, w + Math::Pi, -e);
		ASSERT_VALUES(res, f0);
	}
	{
		auto res = af::FindVelAngle(Q1, w + Math::Pi, -e);
		ASSERT_VALUES(res, f1);
	}
}

TEST_F(firstApproxUtiles_tests, find_pe)
{
	{
		auto [P, E, ok] = af::FindPE(Q0, Q1, r0, r1, w);
		ASSERT_FALSE(ok);
	}
	{
		auto [P, E, ok] = af::FindPE(Q0, Q1, r0, r1, w + Math::Pi);
		ASSERT_TRUE(ok);
		ASSERT_VALUES(P, +p);
		ASSERT_VALUES(E, -e);
	}
}

TEST_F(firstApproxUtiles_tests, find_t)
{
	auto res = af::FindTime(Q0, Q1, M, +p);
	ASSERT_VALUES(res, t);
}

TEST_F(firstApproxUtiles_tests, find_pew)
{
	auto [P, E, W, ok] = af::FindPEW(Q0, Q1, r0, r1, f0);
	ASSERT_TRUE(ok);
	ASSERT_VALUES(W, +w + Math::Pi);
	ASSERT_VALUES(P, +p);
	ASSERT_VALUES(E, -e);
}

TEST_F(firstApproxUtiles_tests, find_pew_noSolution)
{
	auto [P, W, E, ok] = af::FindPEW(Q0, Q1, r0, r1, DEG2RAD(0));
	ASSERT_FALSE(ok);
}

TEST_F(firstApproxUtiles_tests, pathFinder_planarPlanets)
{
	struct TestEphemerides : public IPlanetEphemerides
	{
		float phase0 = 0;
		float period = 0;
		float radius = 0;
		float GM = 0;

		TestEphemerides(float GM, float radius, float period, float phase)
			: period(period)
			, radius(radius)
			, phase0(phase)
			, GM(GM)
		{}

		FVector GetLocation(float time) override
		{
			auto newPhase = phase0 + time / period * 360;
			auto rotation = FQuat({ 0, 0, 1 }, newPhase);
			auto location = FVector(radius, 0, 0);
			return rotation * location;
		}

		float GetGM() const override
		{
			return GM;
		}

		float GetT() const override
		{
			return period;
		}
	};

	struct TestPlanet : public PlanetBase
	{
	public:
		TestPlanet(float GM, float R, float T, float f0)
			: PlanetBase(std::make_shared<TestEphemerides>(GM, R, T, f0))
		{
			SetTime(0);
		}
	};

	auto A = TestPlanet(3.986E+14, 149.6E+9, 31.6E+6, 0 );
	auto B = TestPlanet(4.282E+13, 227.9E+9, 59.4E+6, 20);
	auto C = TestPlanet(1.327E+20, 0, 0, 0);
	auto finder = FirstApproxUtiles::TragectoryFinder(
		A.GetLocation()
		, 0
		, 0
		, B.GetPeriod() / 2
		, B.GetPeriod() / 30
		, C.GetGravParam()
	);
	
	auto ok = finder.FindTragectory(B, DEG2RAD(75));
	ASSERT_TRUE(ok);
	
	auto dt = finder.t1 - finder.t0;
	ASSERT_TRUE(dt > 7.51845e+6 && dt < 7.69125e+6);
}

#include "gtest/gtest.h"
#include "keplerOrbit.hpp"
#include "math/math.hpp"
#include "boost/format.hpp"

namespace af = Pathfinder::Kepler::Elliptic;


struct Kepler_tests : public testing::Test
{
	//! locations
	const FReal r0 = 1.496E+11;
	const FReal r1 = 2.279E+11;
	const FReal Q0 = DEG2RAD(10 );	const FReal q0 = 4.71557;	const FReal E0 = 5.08833;	const FReal M0 = 5.42727;
	const FReal Q1 = DEG2RAD(260);	const FReal q1 = 9.07889;	const FReal E1 = 8.92365;	const FReal M1 = 8.74859;
	//! velocities
	const FReal f0 = DEG2RAD(120);	const FReal v0 = 31712.8;
	const FReal f1 = 5.92284;		const FReal v1 = 19904.4;
	//! orbit params
	const FReal GM = 1.327E+20;
	const FReal w = -1.74215;
	const FReal e =  0.364393;
	const FReal p =  1.49773E+11;
	const FReal a =  1.72705E+11;
	const FReal t =  2.06935e+07;
	
	const bool bf = true;

	void ASSERT_VALUES(FReal val_, FReal exp_)
	{
		auto val = (boost::format("%1.4d") % val_).str();
		auto exp = (boost::format("%1.4d") % exp_).str();
		ASSERT_EQ(exp, val);
	}
};

TEST_F(Kepler_tests, find_w)
{
	auto res = af::w(r0, r1, Q0, Q1, f0);
	ASSERT_VALUES(res, w);
}

TEST_F(Kepler_tests, find_pe)
{
	{
		auto [e_, p_] = af::ep(r0, r1, q0, q1);
		ASSERT_VALUES(e_, +e);
		ASSERT_VALUES(p_, +p);
	}
	{
		auto [e_, p_] = af::ep(r0, r1, q0 + Math::Pi, q1 + Math::Pi);
		ASSERT_VALUES(e_, -e);
		ASSERT_VALUES(p_, +p);
	}
}

TEST_F(Kepler_tests, find_q)
{
	auto [q0_, q1_] = af::qq(Q0, Q1, w);
	ASSERT_VALUES(q0_, q0);
	ASSERT_VALUES(q1_, q1);
}

TEST_F(Kepler_tests, find_E)
{
	{
		auto res = af::E(q0, r0, e);
		ASSERT_VALUES(res, E0);
	}
	{
		auto res = af::E(q1, r1, e);
		ASSERT_VALUES(res, E1);
	}
}

TEST_F(Kepler_tests, find_M)
{
	{
		auto res = af::M(E0, e);
		ASSERT_VALUES(res, M0);
	}
	{
		auto res = af::M(E1, e);
		ASSERT_VALUES(res, M1);
	}
}

TEST_F(Kepler_tests, find_t)
{
	auto res = af::dt(M0, M1, a, GM, bf);
	ASSERT_VALUES(res, t);
}

TEST_F(Kepler_tests, find_v)
{
	{
		auto res = af::v(q0, e, p, GM);
		ASSERT_VALUES(res, v0);
	}
	{
		auto res = af::v(q1, e, p, GM);
		ASSERT_VALUES(res, v1);
	}
}

TEST_F(Kepler_tests, find_f)
{
	{
		auto res = af::f(Q0, q0, e, bf);
		ASSERT_VALUES(res, f0);
	}
	{
		auto res = af::f(Q1, q1, e, bf);
		ASSERT_VALUES(res, f1);
	}
}

TEST_F(Kepler_tests, find_pew)
{
	auto [res, bOK] = af::epwqq(r0, r1, Q0, Q1, f0);
	ASSERT_TRUE(bOK);
	ASSERT_VALUES(res.e, e);
	ASSERT_VALUES(res.p, p);
	ASSERT_VALUES(res.w, w);
	ASSERT_VALUES(res.q0, q0);
	ASSERT_VALUES(res.q1, q1);
}

TEST_F(Kepler_tests, find_pew_noSolution)
{
	auto [res, bOK] = af::epwqq(r0, r1, Q0, Q1, Q0);
	ASSERT_FALSE(bOK);
}

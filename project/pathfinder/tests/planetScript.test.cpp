#include "gtest/gtest.h"
#include "planetScript.hpp"



struct planetScript_tests : public testing::Test
{};

TEST_F(planetScript_tests, check_planetDBConnection)
{
	using namespace Pathfinder::PlanetScript;
	ASSERT_NO_THROW(
		PlanetScript(EPlanet::eEarth, "2019-01-01, 12:00:00 TDB");
	);
}

TEST_F(planetScript_tests, time)
{
	using namespace Pathfinder::PlanetScript;
	auto ep1 = PlanetScript(EPlanet::eEarth, "2019-08-01, 12:00:00 TDB");
	auto ep2 = PlanetScript(EPlanet::eEarth, "2019-01-01, 12:00:00 TDB");
	auto r1 = ep1.GetLocation(0);
	auto r2 = ep2.GetLocation(18316800);
	auto dd = Math::Angle2(r1, r2);
	auto dr = Math::Abs(r2.Size() /r1.Size() - 1);
	
	EXPECT_NEAR(dd, 0, DEG2RAD(0.1));
	EXPECT_NEAR(dr, 0, 0.01);
}

TEST_F(planetScript_tests, getT)
{
	using namespace Pathfinder::PlanetScript;
	auto ep = PlanetScript(EPlanet::eEarth, "2019-01-01, 12:00:00 TDB");
	auto T = ep.GetT(0);
	constexpr auto Tmin = 3600 * 24 * 365 * 0.95;
	constexpr auto Tmax = 3600 * 24 * 366 * 1.05;
	ASSERT_TRUE(Tmin < T && T < Tmax);
}

TEST_F(planetScript_tests, getGM)
{
	using namespace Pathfinder::PlanetScript;
	auto normalize = [](auto v) { return int(v) / 1e10f; };

	auto ep = PlanetScript(EPlanet::eEarth, "2019-01-01, 12:00:00 TDB");
	auto GM = ep.GetGM(0);
	auto ex = 3.986E+14;
	ASSERT_FLOAT_EQ(normalize(GM), normalize(ex));
}

TEST_F(planetScript_tests, getLocation)
{
	using namespace Pathfinder::PlanetScript;
	auto ep = PlanetScript(EPlanet::eEarth, "2019-01-01, 12:00:00 TDB");
	auto l1 = ep.GetLocation(0);
	auto l2 = ep.GetLocation(3600 * 24 * 30);
	ASSERT_NE(l1, l2);
}

TEST_F(planetScript_tests, getVelocity)
{
	using namespace Pathfinder::PlanetScript;
	auto ep = PlanetScript(EPlanet::eEarth, "2019-01-01, 12:00:00 TDB");
	auto v1 = ep.GetVelocity(0);
	auto v2 = ep.GetVelocity(3600 * 24 * 30);
	ASSERT_NE(v1, v2);
	EXPECT_NEAR(v1.Size(), 29746, 1e+3);
	EXPECT_NEAR(v2.Size(), 29746, 1e+3);
}

TEST_F(planetScript_tests, VelocityOrientation)
{
	using namespace Pathfinder::PlanetScript;
	auto ep = PlanetScript(EPlanet::eEarth, "2019-01-01, 12:00:00 TDB");
	auto v1 = ep.GetVelocity(0);
	auto r1 = ep.GetLocation(0);
	auto a = Math::Angle2(r1, v1);
	ASSERT_NEAR(a, DEG2RAD(90), DEG2RAD(5));
}

TEST_F(planetScript_tests, DiscreteMode)
{
	using namespace Pathfinder::PlanetScript;
	auto epc = PlanetScript(EPlanet::eEarth, "2019-01-01, 12:00:00 TDB");
	auto epd = PlanetScript(EPlanet::eEarth, "2019-01-01, 12:00:00 TDB");

	// \note: expected structure
	// n  0   1   2   3   3.(3)-  3.(3)+  4
	// N  0   0   0   0     0       1     1
	// t 0.0 3.0 6.0 9.0 (10.0-) (10.0+) 12.0
	auto step  =  3e+4;
	auto chunk = 10e+4; // != step*N, n in Z
	ASSERT_NO_THROW(epd.MakeDiscret(step, chunk));

	for (auto& [td, tc, msg] : std::vector<std::tuple<FReal, FReal, std::string>>{
		  {step * 3, step * 3  , "1st bucket"}
		, {step * 4, step * 4  , "2nd bucket"}
		, {step * 3, step * 3.2, "is a stair like?"}
		, {step * 3, step * 3.4, "is the same stair?"}
	})
	{
		auto [rc, vc] = epc.GetMovement(tc);
		auto [rd, vd] = epd.GetMovement(td);
		auto ar = Math::Angle2(rc, rd);
		auto av = Math::Angle2(vc, vd);
		EXPECT_LE(ar, DEG2RAD(1)) << msg;
		EXPECT_LE(av, DEG2RAD(1)) << msg;
	}
}

#if 0
#include <fstream>
TEST_F(planetScript_tests, ExportOrbit)
{
	using namespace Pathfinder::PlanetScript;
	auto ep = PlanetScript(EPlanet::eEarth, "2020-01-07, 12:00:00 TDB");
	auto os = std::ofstream("D:/(HW)/(HW)/8 - Diploma/math/tmp.json");
	if (!os)
	{
		abort();
	}
	
	os << "[" << std::endl;
	auto T = ep.GetT(0);
	for (auto t = 0; t < 2*T; t += 3600 * 24)
	{
		auto r = ep.GetLocation(t);
		os << "{ \"t\":" << t << ", \"r\": [" << r.x << ", " << r.y << ", " << r.z << "]}, " << std::endl;
	}
	os << "{}" << std::endl;
	os << "]";
}
#endif

#if 0
TEST_F(planetScript_tests, CoordSource)
{
	using namespace Pathfinder::PlanetScript;
	auto ep0 = PlanetScript(EPlanet::eEarth, "2011-11-26, 12:00:00 TDB");
	auto ep1 = PlanetScript(EPlanet::eMars , "2012-08-06, 12:00:00 TDB");
	auto [r0, v0] = ep0.GetMovement(0);
	auto [r1, v1] = ep1.GetMovement(0);

	r1 != r0;
}
#endif

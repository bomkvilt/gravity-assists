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
	ASSERT_FALSE(l1 == l2);
}

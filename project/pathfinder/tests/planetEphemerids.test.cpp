#include "gtest/gtest.h"
#include "planetEphemerides.hpp"



struct planetEphemerides_tests : public testing::Test
{};


TEST_F(planetEphemerides_tests, check_planetDBConnection)
{
	ASSERT_NO_THROW(
		PlanetEphemerides(EPlanet::eEarth, "2019-01-01, 12:00:00 TDB");
	);
}

TEST_F(planetEphemerides_tests, getT)
{
	auto ep = PlanetEphemerides(EPlanet::eEarth, "2019-01-01, 12:00:00 TDB");
	auto T = ep.GetT();
	constexpr auto Tmin = 3600 * 24 * 365 * 0.95;
	constexpr auto Tmax = 3600 * 24 * 366 * 1.05;
	ASSERT_TRUE(Tmin < T && T < Tmax);
}

TEST_F(planetEphemerides_tests, getGM)
{
	auto normalize = [](auto v) { return int(v) / 1e10f; };

	auto ep = PlanetEphemerides(EPlanet::eEarth, "2019-01-01, 12:00:00 TDB");
	auto GM = ep.GetGM();
	auto ex = 3.986E+14;
	ASSERT_FLOAT_EQ(normalize(GM), normalize(ex));
}

TEST_F(planetEphemerides_tests, getLocation)
{
	auto ep = PlanetEphemerides(EPlanet::eEarth, "2019-01-01, 12:00:00 TDB");
	auto l1 = ep.GetLocation(0);
	auto l2 = ep.GetLocation(3600 * 24 * 30);
	ASSERT_FALSE(l1 == l2);
}

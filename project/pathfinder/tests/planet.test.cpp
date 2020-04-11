#include "gtest/gtest.h"
#include "planet.hpp"
#include "planetEphemerides.hpp"



struct planet_tests : public testing::Test
{};


TEST_F(planet_tests, check_planetDBConnection)
{
	constexpr auto initialTime = "2019-01-01, 12:00:00 TDB";
	auto ep = PlanetEphemerides(EPlanet::eEarth, initialTime);
	auto pl = Planet(EPlanet::eEarth, initialTime);

	ASSERT_EQ(ep.GetGM(), pl.GetGravParam());

	pl.SetTime(1e6);
	ASSERT_EQ(ep.GetLocation(1e6), pl.GetLocation());
}

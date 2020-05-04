#include "gtest/gtest.h"
#include "interfaces/ephemerides.hpp"
#include "planetScriptSimple.hpp"
#include "blocks/link.hpp"


struct Link_tests : public testing::Test
{};


TEST_F(Link_tests, cocentricOrbits_venus)
{
	using namespace Pathfinder;
	// flight from the Earth to the Venus with initial phase distance of 240 deg.
	auto A = PlanetScript::PlanetScriptSimple(3.986E+14, 149.6E+9, 31.6E+6, DEG2RAD(0  ));
	auto B = PlanetScript::PlanetScriptSimple(3.248E+14, 108.2E+9, 19.4E+6, DEG2RAD(240));
	auto C = PlanetScript::PlanetScriptSimple(1.327E+20, 0, 0, 0);
	auto conf = Link::ScriptedLinkConfig();
	conf.t0 = 0;
	conf.SetA(A);
	conf.SetB(B);
	conf.te = B.GetT(0);
	conf.ts = B.GetT(0) / 160;
	conf.tt = 3600 * 24;
	conf.td = 3600 * 24 / 100;
	conf.GM = C.GetGM(0);
	// find all roots sutable for 70 deg (+20deg to local horisont)
	auto links = std::vector<Link::Link>();
	Link::FindLinks(links, conf, { DEG2RAD(70) });
	
	ASSERT_EQ(links.size(), 1);
	EXPECT_NEAR(links[0].t1, 1.25e+7, 0.05e+7);
}

TEST_F(Link_tests, cocentricOrbits_mars)
{
	using namespace Pathfinder;
	// flight from the Earth to the Mars with initial phase distance of 0 deg.
	auto A = PlanetScript::PlanetScriptSimple(3.986E+14, 149.6E+9, 31.6E+6, DEG2RAD(0));
	auto B = PlanetScript::PlanetScriptSimple(4.282E+13, 227.9E+9, 59.4E+6, DEG2RAD(0));
	auto C = PlanetScript::PlanetScriptSimple(1.327E+20, 0, 0, 0);
	auto conf = Link::ScriptedLinkConfig();
	conf.t0 = 0;
	conf.SetA(A);
	conf.SetB(B);
	conf.te = B.GetT(0);
	conf.ts = B.GetT(0) / 160;
	conf.tt = 3600 * 24;
	conf.td = 3600 * 24 / 100;
	conf.GM = C.GetGM(0);
	// find all roots sutable for 70 deg (+20deg to local horisont)
	auto links = std::vector<Link::Link>();
	Link::FindLinks(links, conf, { DEG2RAD(70) });

	ASSERT_EQ(links.size(), 1);
	EXPECT_NEAR(links[0].t1, 1.75e+7, 0.1e+7);
}

TEST_F(Link_tests, cocentricOrbits_tangency)
{
	using namespace Pathfinder;
	// flight from the Earth to the Mars with initial phase distance of 0 deg.
	auto A = PlanetScript::PlanetScriptSimple(3.986E+14, 149.6E+9, 31.6E+6, 0.);
	auto B = PlanetScript::PlanetScriptSimple(4.282E+13, 227.9E+9, 59.4E+6, 0.776);
	auto C = PlanetScript::PlanetScriptSimple(1.327E+20, 0, 0, 0);
	auto conf = Link::ScriptedLinkConfig();
	conf.t0 = 0;
	conf.SetA(A);
	conf.SetB(B);
	conf.te = B.GetT(0);
	conf.ts = B.GetT(0) / 160;
	conf.tt = 3600 * 24;
	conf.td = 3600 * 24 / 100;
	conf.GM = C.GetGM(0);
	
	auto links = std::vector<Link::Link>();
	Link::FindLinks(links, conf, { DEG2RAD(90) });

	ASSERT_EQ(links.size(), 5);
	EXPECT_NEAR(links[4].t1, 2.23e+7, 0.1e+7);
	EXPECT_NEAR(links[4].v0, 32726, 1e+2);
	EXPECT_NEAR(links[4].v1, 21482, 1e+2);
}

TEST_F(Link_tests, 3DVelocity)
{
	namespace l = Pathfinder::Link;
	namespace u = Pathfinder::Link::Utiles;
	auto finder = u::ScriptedLink(l::ScriptedLinkConfig(), 0);
	// trajectory in xz-plane
	finder.R0 = FVector(10, 0, 0);
	finder.R1 = FVector(0, 0, 10);
	finder.f0 = DEG2RAD( 90);
	finder.f1 = DEG2RAD(180);
	finder.v0 = 60;
	finder.v1 = 60;
	finder.Fix3DParams();
	
	constexpr auto epsilon = 3 * Math::Epsilon;
	EXPECT_NEAR(finder.V0.x,  0, epsilon);
	EXPECT_NEAR(finder.V0.y,  0, epsilon);
	EXPECT_NEAR(finder.V0.z, 60, epsilon);
	EXPECT_NEAR(finder.V1.x,-60, epsilon);
	EXPECT_NEAR(finder.V1.y,  0, epsilon);
	EXPECT_NEAR(finder.V1.z,  0, epsilon);
}

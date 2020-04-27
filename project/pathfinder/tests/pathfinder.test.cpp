#include "gtest/gtest.h"
#include "pathfinder.hpp"
#include "planetScript.hpp"
#include "planetScriptSimple.hpp"



struct pathfinder_tests : public testing::Test
{};


TEST_F(pathfinder_tests, circularOrbits)
{
	using namespace Pathfinder;

	auto scripts = std::vector{
		std::make_shared<PlanetScript::PlanetScriptSimple>(1.327E+20, 0, 0, 0),
		std::make_shared<PlanetScript::PlanetScriptSimple>(3.986E+14, 149.6E+9, 31.6E+6, 0.0),
		std::make_shared<PlanetScript::PlanetScriptSimple>(4.282E+13, 249.2E+9, 59.4E+6, 3.0)
	};

	auto A = std::make_unique<NodeDeparture>();
	auto B = std::make_unique<NodeArrival>();
	A->EscapeSpeed = 11e+11;
	B->ParkingRadius = 3.8e+6;
	B->SphereRadius = 1.3e+8;
	A->Script = scripts[1];
	B->Script = scripts[2];

	auto mission = Mission();
	mission.GM = scripts[0]->GetGM(0);
	mission.normalFlyPeriodFactor = 1;
	mission.points_f0 = 60;
	mission.timeStep = 3600 * 24 * 15;
	mission.timeTol = 3600 * 24;
	mission.t0 = 0;
	mission.nodes.push_back(std::move(A));
	mission.nodes.push_back(std::move(B));

	auto solver = PathFinder(std::move(mission));
	auto paths = solver.FirstApprox();

	ASSERT_EQ(paths.size(), 15);
}

TEST_F(pathfinder_tests, realPlanets)
{
	using namespace Pathfinder;
	// this date is a mid of a E-M launch window
	constexpr auto startDate = "2020-01-06, 12:00:00 TDB";

	auto scripts = std::vector{
		std::make_shared<PlanetScript::PlanetScript>(PlanetScript::EPlanet::eSun   , startDate),
		std::make_shared<PlanetScript::PlanetScript>(PlanetScript::EPlanet::eEarth , startDate),
		std::make_shared<PlanetScript::PlanetScript>(PlanetScript::EPlanet::eJupter, startDate)
	};

	auto A = std::make_unique<NodeDeparture>();
	auto B = std::make_unique<NodeArrival>();
	A->EscapeSpeed   = 11e+11;
	B->ParkingRadius = 3.8e+6;
	B->SphereRadius  = 1.3e+8;
	A->Script = scripts[1];
	B->Script = scripts[2];

	auto mission = Mission();
	mission.GM = scripts[0]->GetGM(0);
	mission.normalFlyPeriodFactor = 1;
	mission.points_f0 = 60;
	mission.timeStep = 3600*24*15;
	mission.timeTol = 3600*24;
	mission.t0 = 0;
	mission.nodes.push_back(std::move(A));
	mission.nodes.push_back(std::move(B));

	auto solver = PathFinder(std::move(mission));
	auto paths  = solver.FirstApprox();

	ASSERT_EQ(paths.size(), 15);
}

#include "configs/problemConfig.hpp"
#include "planetScript.hpp"



namespace AXConf_::Utiles
{
	const FReal& Check(const FReal& val, const std::string& name)
	{
		if (!isnan(val))
		{
			return val;
		}
		throw std::runtime_error("field '" + name + "' must be set.");
	}
}

#define AX_CONF_CHECK(field)						\
	AXConf_::Utiles::Check(field, type + "."#field)	\
/**/



void TimeConfig::CheckIsValid() const
{
	const auto type = std::string("Mission.TimeSettings");
	AX_CONF_CHECK(discretisation);
	AX_CONF_CHECK(chunkSize);
}


Pathfinder::MissionConfig AXConf::MakeConfig(const TimeConfig& tconf) const
{
	using Pathfinder::MissionConfig;

	auto conf = MissionConfig();
	conf.normalFlyPeriodFactor = AX_CONF_CHECK(periodFactor);
	conf.points_f0 = AX_CONF_CHECK(points_f0);
	conf.timeFrac = tconf.discretisation;
	conf.timeStep = AX_CONF_CHECK(timeStep);
	conf.timeTol  = AX_CONF_CHECK(timeTol );
	return conf;
}


Pathfinder::FAXConfig FAXConf::MakeConfig(const TimeConfig& tconf) const
{
	type = "Mission.FAX";

	auto conf = Pathfinder::FAXConfig();
	conf.CopyValus(AXConf::MakeConfig(tconf));
	return conf;
}


Pathfinder::SAXConfig SAXConf::MakeConfig(const TimeConfig& tconf) const
{
	type = "Mission.SAX";

	auto conf = Pathfinder::SAXConfig();
	conf.CopyValus(AXConf::MakeConfig(tconf));
	conf.burnArcFraction = AX_CONF_CHECK(burnArcFraction);
	conf.maxMinimisationIters = AX_CONF_CHECK(maxMinimisationIters);
	conf.minMinimisationDelta = AX_CONF_CHECK(minMinimisationDelta);
	conf.initialBurnPointStep = AX_CONF_CHECK(initialBurnPointStep);
	conf.initialTossAngleStep = AX_CONF_CHECK(initialTossAngleStep);
	conf.initialTimeStep = AX_CONF_CHECK(initialTimeStep);
	conf.burnNodeFactory = [
		  i = AX_CONF_CHECK(burnImpulseLimit)
		, a = AX_CONF_CHECK(burnImpulse_a)
		, k = AX_CONF_CHECK(burnImpulse_k)
	]() {
		auto ptr = std::make_shared<Pathfinder::Nodes::BurnNode>();
		ptr->ImpulseLimit = i;
		ptr->A_impulse = a;
		ptr->K_impulse = k;
		return ptr;
	};
	return conf;
}


FunctionalityConfig::Functionality FunctionalityConfig::MakeFunctionality() const
{
	using Pathfinder::PathFinder;

	const auto type = std::string("Mission.Functionality");

	return [
		  c = AX_CONF_CHECK(correction)
		, m = AX_CONF_CHECK(mismatch)
		, i = AX_CONF_CHECK(impulse)
		, t = AX_CONF_CHECK(time)
	](const PathFinder::FlightChain& flight) {
		return c * flight.Correction
			+  m * flight.Mismatch
			+  i * flight.Impulse
			+  t * flight.totalTime;
	};
}


Pathfinder::Mission ProblemConfig::MakeMission() const
{
	using Pathfinder::PlanetScript::PlanetScript;
	using Pathfinder::PlanetScript::EPlanet;

	const auto sun = PlanetScript(EPlanet::eSun, timeSettings.startDate);

	auto mission = Pathfinder::Mission();
	mission.faxConfig = faxConf.MakeConfig(timeSettings);
	mission.saxConfig = saxConf.MakeConfig(timeSettings);
	mission.t0 = timeSettings.t0;
	mission.GM = sun.GetGM(0);
	for (auto planet : planets)
	{
		mission.nodes.push_back(planet.ProduceNode(timeSettings));
	}
	return mission;
}

Pathfinder::PathFinder ProblemConfig::MakeFinder() const
{
	const auto type = std::string("Mission");

	timeSettings.CheckIsValid();

	AX_CONF_CHECK(keepFactor);
	if (!(keepFactor > 0 && keepFactor <= 1))
	{
		throw std::runtime_error(type + "keepFactor must be in range of (0, 1]");
	}

	auto finder = Pathfinder::PathFinder(MakeMission());
	finder.SetFunctionality(functionality.MakeFunctionality());
	return finder;
}

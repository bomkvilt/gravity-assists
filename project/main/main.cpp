#include "configs/mainConfig.hpp"
#include "handlers/makeMainConf.hpp"
#include "handlers/makeProblemConfig.hpp"
#include "handlers/traceTrajectory.hpp"
#include "handlers/tracePlanet.hpp"
#include "handlers/problemSolver.hpp"
#include "planetScript.hpp"

#include <gflags/gflags.h>
#include <iostream>


DEFINE_string(coreConfPath, "./core.json", "path to pathfinder core configuration");
DEFINE_string(f           , ""           , "path to mission config");
DEFINE_string(makeProbConf, ""           , "path to make default mission configuration file");
DEFINE_string(makeCoreConf, ""           , "path to make core configuration file");

DEFINE_string(tracePath, "", "");
DEFINE_double(taceFraction, 0.1, "");
DEFINE_string(tracePlanet, "", "");
DEFINE_string(traceOrigin, "", "");
DEFINE_double(traceBgn, 0, "");
DEFINE_double(traceEnd, 0, "");
DEFINE_double(traceStep, 0, "");
DEFINE_string(o, "", "");



int main(int argc, char** argv)
{
	gflags::SetUsageMessage("\n"
		"[ -coreConfPath=\"./pf_conf.json\" ] \n"
		"[ -f=\"path_to_problem_conf.json\" ] \n"
		"[ -makeProbConf=\"path_to_conf\"   ] \n"
		"[ -makeCoreConf=\"path_to_conf\"   ] \n"
	);

	if (argc == 1)
	{
		gflags::ShowUsageWithFlags(argv[0]);
		std::exit(1);
	}
	try
	{
		gflags::ParseCommandLineFlags(&argc, &argv, true);

		if (FLAGS_makeProbConf.size())
		{
			return makeProblemConf(FLAGS_makeProbConf);
		}
		if (FLAGS_makeCoreConf.size())
		{
			return MakeMainConf(FLAGS_makeCoreConf);
		}
		if (FLAGS_o.size() && FLAGS_tracePath.size() && FLAGS_taceFraction > 0 && FLAGS_taceFraction <= 1)
		{
			return TraceTrajectory(FLAGS_o, FLAGS_tracePath, FLAGS_taceFraction);
		}
		
		auto mainConfig = MainConfig();
		if (!mainConfig.LoadConfig(FLAGS_coreConfPath))
		{
			std::cout << "config file cannot be parsed" << std::endl;
		}
		else // init enviroment
		{
			Pathfinder::PlanetScript::InitDatabases(mainConfig.kernelPath);
		}
		
		if (FLAGS_o.size() && FLAGS_tracePlanet.size() && FLAGS_traceOrigin.size() && FLAGS_traceBgn > 0 && FLAGS_traceEnd && FLAGS_traceStep > 0)
		{
			return TracePlanet(FLAGS_o, FLAGS_tracePlanet, FLAGS_traceOrigin, FLAGS_traceBgn, FLAGS_traceEnd, FLAGS_traceStep);
		}

		if (FLAGS_f.size())
		{
			return SolveProblem(FLAGS_f);
		}
		
		gflags::ShowUsageWithFlags(argv[0]);
		std::exit(1);
	}
	catch (const std::exception& e)
	{
		std::cerr << "Unexpected exception:" << std::endl;
		std::cerr << e.what() << std::endl;
	}
}

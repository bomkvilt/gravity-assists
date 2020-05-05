#include "configs/mainConfig.hpp"
#include "handlers/makeMainConf.hpp"
#include "handlers/makeProblemConfig.hpp"
#include "handlers/problemSolver.hpp"
#include "planetScript.hpp"

#include <gflags/gflags.h>
#include <iostream>

DEFINE_string(mainConfPath, "pf_conf.json", "path to pathfinder core configuration");
DEFINE_string(f           , ""            , "path to mission config");
DEFINE_string(makeProbConf, ""            , "path to make default mission configuration file");
DEFINE_string(makeCoreConf, ""            , "path to make core configuration file");



int main(int argc, char** argv)
{
	gflags::ParseCommandLineFlags(&argc, &argv, true);

	auto mainConfig = MainConfig();
	if (!mainConfig.LoadConfig(FLAGS_mainConfPath))
	{
		std::cout << "config file cannot be parsed" << std::endl;
	}
	else // init enviroment
	{
		Pathfinder::PlanetScript::InitDatabases(mainConfig.kernelPath);
	}

	if (FLAGS_makeProbConf.size())
	{
		return makeProblemConf(FLAGS_makeProbConf);
	}
	if (FLAGS_makeCoreConf.size())
	{
		return MakeMainConf(FLAGS_makeCoreConf);
	}
	if (FLAGS_f.size())
	{
		return SolveProblem(FLAGS_f);
	}

	gflags::ShowUsageWithFlags(argv[0]);
	std::exit(1);
}

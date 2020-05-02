#include "pathfinder.hpp"
#include "planetScript.hpp"
#include "mission.hpp"

#include "mainConfig.hpp"

#include <gflags/gflags.h>
#include <iostream>

DEFINE_string(mainConfPath, "pf_conf.json", "path to pathfinder core configuration");
DEFINE_string(f, "", "path to symulation config");


int main(int argc, char** argv)
{
	gflags::ParseCommandLineFlags(&argc, &argv, true);

	auto mainConfig = MainConfig();
	if (!mainConfig.LoadConfig(FLAGS_mainConfPath))
	{
		std::cout << "config file cannot be parsed" << std::endl;
	}

	// init SPICE kernels
	Pathfinder::PlanetScript::InitDatabases(mainConfig.kernelPath);


}

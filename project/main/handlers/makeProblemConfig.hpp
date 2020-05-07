#ifndef MAIN__MAKEPROBLEMCONFIG_HPP
#define MAIN__MAKEPROBLEMCONFIG_HPP

#include "configs/problemConfig.hpp"
#include <iostream>



int makeProblemConf(const std::string& path)
{
	if (std::filesystem::is_directory(path))
	{
		throw std::runtime_error("Passed path is directory. FileName was required.");
	}

	auto conf = ProblemConfig();
	conf.planets.resize(1);
	if (!conf.SaveConfig(path))
	{
		throw std::runtime_error("Unknown error during file saveing: '" + path + "'");
	}
	std::cout << " -- Problem config was placed to " << path << std::endl;
	return 0;
}


#endif //!MAIN__MAKEPROBLEMCONFIG_HPP

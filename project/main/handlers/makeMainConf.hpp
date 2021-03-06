#ifndef MAIN__MAKEMAINCONF_HPP
#define MAIN__MAKEMAINCONF_HPP

#include "configs/mainConfig.hpp"
#include <filesystem>
#include <iostream>



int MakeMainConf(const std::string& path)
{
	if (std::filesystem::is_directory(path))
	{
		throw std::runtime_error("Passed path is directory. FileName was required.");
	}

	auto conf = MainConfig();
	if (!conf.SaveConfig(path))
	{
		throw std::runtime_error("Unknown error during file saveing: '" + path + "'");
	}
	std::cout << " -- Core config was placed to " << path << std::endl;
	return 0;
}


#endif //!MAIN__MAKEMAINCONF_HPP

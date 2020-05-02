#ifndef MAIN__MAINCONFIG_HPP
#define MAIN__MAINCONFIG_HPP

#include "reflect/config.hpp"


struct MainConfig : public reflect::FConfig
{
	ARCH_BEGIN(reflect::FConfig)
		ARCH_END()
public:
	std::string kernelPath = SPICE_KERNELS;
};


#endif //!MAIN__MAINCONFIG_HPP

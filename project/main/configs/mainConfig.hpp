#ifndef MAIN__MAINCONFIG_HPP
#define MAIN__MAINCONFIG_HPP

#include "reflect/config.hpp"



struct MainConfig : public reflect::FConfig
{
	ARCH_BEGIN(reflect::FConfig)
		ARCH_FIELD(, , kernelPath)
		ARCH_END()
public:
	std::string kernelPath;
};


#endif //!MAIN__MAINCONFIG_HPP

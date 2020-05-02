#ifndef MAIN__PROBLEMCONFIG_HPP
#define MAIN__PROBLEMCONFIG_HPP

#include "reflect/config.hpp"


struct ProblemConfig : public reflect::FConfig
{
	ARCH_BEGIN(reflect::FConfig)
		ARCH_FIELD(, , planets)
		ARCH_END()
public:
	std::vector<std::string> planets;
};


#endif //!MAIN__PROBLEMCONFIG_HPP

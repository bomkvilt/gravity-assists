#ifndef MAIN__TIMECONFIG_HPP
#define MAIN__TIMECONFIG_HPP

#include "reflect/config.hpp"
#include "math/math.hpp"



struct TimeConfig : public reflect::FConfig
{
	ARCH_BEGIN(reflect::FConfig)
		ARCH_FIELD(, , startDate)
		ARCH_FIELD(, , discretisation)
		ARCH_FIELD(, , chunkSize)
		ARCH_FIELD(, , t0)
		ARCH_FIELD(, , t1)
		ARCH_FIELD(, , dt)
		ARCH_END()
public:
	std::string startDate;
	FReal discretisation = NAN;
	FReal chunkSize      = NAN;

	FReal t0 = NAN;
	FReal t1 = NAN; 
	FReal dt = NAN;

	void CheckIsValid() const;
};


#endif //!MAIN__TIMECONFIG_HPP

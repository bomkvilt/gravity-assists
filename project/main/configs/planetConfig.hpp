#ifndef MAIN__PLANETCONFIG_HPP
#define MAIN__PLANETCONFIG_HPP

#include "configs/timeConfig.hpp"
#include "pathfinder.hpp"
#include "nodes.hpp"



struct PlanetConfig : public reflect::FConfig
{
	ARCH_BEGIN(reflect::FConfig)
		ARCH_FIELD(, , planet)
		ARCH_FIELD(, , nodeType)

		ARCH_FIELD(, , sphereRarius)
		ARCH_FIELD(, , parkingRadius)
		ARCH_FIELD(, , energyConstant)

		ARCH_FIELD(, , mismatchLimit)
		ARCH_FIELD(, , mismatch_a)
		ARCH_FIELD(, , mismatch_k)
		
		ARCH_FIELD(, , impulseLimit)
		ARCH_FIELD(, , impulse_a)
		ARCH_FIELD(, , impulse_k)

		ARCH_FIELD(, , kink_a)
		ARCH_FIELD(, , kink_k)
		ARCH_END()
public:
	std::string planet;
	std::string nodeType;

	FReal sphereRarius = NAN;
	FReal parkingRadius = NAN;
	FReal energyConstant = NAN;
	
	FReal mismatchLimit = 0;
	FReal mismatch_a = 1;
	FReal mismatch_k = 4;
	
	FReal impulseLimit  = 0;
	FReal impulse_a = 1;
	FReal impulse_k = 4;

	FReal kink_a = 1;
	FReal kink_k = 1;

public:
	Pathfinder::Nodes::INode::ptr ProduceNode(const TimeConfig& deskConf) const;
};


#endif //!MAIN__PLANETCONFIG_HPP

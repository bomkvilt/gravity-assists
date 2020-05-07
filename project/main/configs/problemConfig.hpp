#ifndef MAIN__PROBLEMCONFIG_HPP
#define MAIN__PROBLEMCONFIG_HPP

#include "configs/planetConfig.hpp"



struct AXConf : public reflect::FConfig
{
	ARCH_BEGIN(reflect::FConfig)
		ARCH_FIELD(, , periodFactor)
		ARCH_FIELD(, , points_f0)
		ARCH_FIELD(, , timeStep)
		ARCH_FIELD(, , timeTol)
		ARCH_END()
public:
	FReal periodFactor = NAN;
	FReal points_f0 = NAN;
	FReal timeStep = NAN;
	FReal timeTol = NAN;

	Pathfinder::MissionConfig MakeConfig(const TimeConfig& tconf) const;

protected:
	mutable std::string type;
};


struct FAXConf : public AXConf
{
	Pathfinder::FAXConfig MakeConfig(const TimeConfig& tconf) const;
};


struct SAXConf : public AXConf
{
	ARCH_BEGIN(AXConf)
		ARCH_FIELD(, , burnArcFraction)
		ARCH_FIELD(, , minMinimisationDelta)
		ARCH_FIELD(, , maxMinimisationIters)
		ARCH_FIELD(, , initialTossAngleStep)
		ARCH_FIELD(, , initialBurnPointStep)
		ARCH_FIELD(, , initialTimeStep)
		ARCH_FIELD(, , burnImpulseLimit)
		ARCH_FIELD(, , burnImpulse_a)
		ARCH_FIELD(, , burnImpulse_k)
		ARCH_END()
public:

	FReal burnArcFraction = 0.5;
	FReal minMinimisationDelta = 0.1;
	Int32 maxMinimisationIters = 100;

	FReal initialTossAngleStep = 0.001;
	FReal initialBurnPointStep = 1.e+6;
	FReal initialTimeStep = 3600. * 12;

	FReal burnImpulseLimit = NAN;
	FReal burnImpulse_a = 1;
	FReal burnImpulse_k = 4;

public:

	Pathfinder::SAXConfig MakeConfig(const TimeConfig& tconf) const;
};


struct FunctionalityConfig : public reflect::FConfig
{
	ARCH_BEGIN(reflect::FConfig)
		ARCH_FIELD(, , correction)
		ARCH_FIELD(, , mismatch)
		ARCH_FIELD(, , impulse)
		ARCH_FIELD(, , time)
		ARCH_END()
public:

	FReal correction = NAN;
	FReal mismatch = NAN;
	FReal impulse = NAN;
	FReal time = NAN;

public:
	using Functionality = Pathfinder::PathFinder::Functionality;

	Functionality MakeFunctionality() const;
};


struct ProblemConfig : public reflect::FConfig
{
	ARCH_BEGIN(reflect::FConfig)
		ARCH_FIELD(, , functionality)
		ARCH_FIELD(, , timeSettings)
		ARCH_FIELD(, , planets)
		ARCH_FIELD(, , faxConf)
		ARCH_FIELD(, , saxConf)
		ARCH_FIELD(, , keepFactor)
		ARCH_END()
public:

	using Planets = std::vector<PlanetConfig>;

public:

	FunctionalityConfig functionality;

	TimeConfig timeSettings;
	Planets planets;

	FAXConf faxConf;
	SAXConf saxConf;

	FReal keepFactor = NAN;

private:

	Pathfinder::Mission MakeMission() const;

public:

	Pathfinder::PathFinder MakeFinder() const;
};


#endif //!MAIN__PROBLEMCONFIG_HPP

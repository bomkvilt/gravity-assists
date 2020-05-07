#include "configs/planetConfig.hpp"
#include "planetScript.hpp"
#include  <boost/algorithm/string.hpp>



namespace PlanetConfig_::Utiles
{
	using Pathfinder::PlanetScript::EPlanet;
	using Pathfinder::PlanetScript::PlanetScript;

	EPlanet GetPlanetName(const std::string& name_)
	{
		auto name = boost::to_lower_copy(name_);
		if (name == "mercury") return EPlanet::eMercury;
		if (name == "venus"  ) return EPlanet::eVenus;
		if (name == "earth"  ) return EPlanet::eEarth;
		if (name == "mars"   ) return EPlanet::eMars;
		if (name == "jupter" ) return EPlanet::eJupter;
		if (name == "moon"   ) return EPlanet::eMoon;
		throw std::runtime_error("unexpected planet name: " + name);
	}

	PlanetScript::ptr CreatePlanetScript(const std::string& planetName, const std::string& startDate)
	{
		return std::make_shared<PlanetScript>(GetPlanetName(planetName), startDate);
	}



	enum class ENodeType
	{
		  eDeparture_Circular
		, eDeparture_Energy
		, eArrival_Circular
		, eArrival_Energy
		, eFlyBy
	};

	ENodeType GetNodeType(const std::string& type_)
	{
		auto type = boost::to_lower_copy(type_);
		if (type == "departure.circular") return ENodeType::eDeparture_Circular;
		if (type == "departure.energy"  ) return ENodeType::eDeparture_Energy;
		if (type == "arrival.circular"  ) return ENodeType::eArrival_Circular;
		if (type == "arrival.energy"    ) return ENodeType::eArrival_Energy;
		if (type == "flyby"             ) return ENodeType::eFlyBy;
		throw std::runtime_error("unexpected node type: " + type);
	}

	const FReal& Check(const FReal& val, const std::string& name)
	{
		if (!isnan(val))
		{
			return val;
		}
		throw std::runtime_error("field '" + name + "' must be set.");
	}

}

#define PLAN_CONF_CHECK(field)										\
	Utiles::Check(field, "Mission.Planets" + planet + "."#field)	\
/**/



Pathfinder::Nodes::INode::ptr PlanetConfig::ProduceNode(const TimeConfig& deskConf) const
{
	using namespace PlanetConfig_;
	using namespace Pathfinder;
	
	auto script = Utiles::CreatePlanetScript(planet, deskConf.startDate);
	script->MakeDiscret(deskConf.discretisation, deskConf.chunkSize);


	switch (Utiles::GetNodeType(nodeType)) {
	case Utiles::ENodeType::eDeparture_Circular: 
	{
		auto node = std::make_unique<NodeDeparture::Circular>();
		node->ParkingRadius = PLAN_CONF_CHECK(parkingRadius);
		node->SphereRadius = PLAN_CONF_CHECK(sphereRarius);
		
		node->ImpulseLimit = PLAN_CONF_CHECK(impulseLimit);
		node->A_impulse = PLAN_CONF_CHECK(impulse_a);
		node->K_impulse = PLAN_CONF_CHECK(impulse_k);
		
		node->Script = script;
		return node;
	}

	case Utiles::ENodeType::eDeparture_Energy:
	{
		auto node = std::make_unique<NodeDeparture::EnergyDriven>();
		node->ParkingRadius = PLAN_CONF_CHECK(parkingRadius);
		node->SphereRadius = PLAN_CONF_CHECK(sphereRarius);
		node->h0 = PLAN_CONF_CHECK(energyConstant);

		node->ImpulseLimit = PLAN_CONF_CHECK(impulseLimit);
		node->A_impulse = PLAN_CONF_CHECK(impulse_a);
		node->K_impulse = PLAN_CONF_CHECK(impulse_k);

		node->Script = script;
		return node;
	}
	
	case Utiles::ENodeType::eArrival_Circular:
	{
		auto node = std::make_unique<NodeArrival::Circular>();
		node->ParkingRadius = PLAN_CONF_CHECK(parkingRadius);
		node->SphereRadius = PLAN_CONF_CHECK(sphereRarius);
		
		node->ImpulseLimit = PLAN_CONF_CHECK(impulseLimit);
		node->A_impulse = PLAN_CONF_CHECK(impulse_a);
		node->K_impulse = PLAN_CONF_CHECK(impulse_k);

		node->Script = script;
		return node;
	}

	case Utiles::ENodeType::eArrival_Energy:
	{
		auto node = std::make_unique<NodeArrival::EnergyDriven>();
		node->ParkingRadius = PLAN_CONF_CHECK(parkingRadius);
		node->SphereRadius = PLAN_CONF_CHECK(sphereRarius);
		node->h1 = PLAN_CONF_CHECK(energyConstant);

		node->ImpulseLimit = PLAN_CONF_CHECK(impulseLimit);
		node->A_impulse = PLAN_CONF_CHECK(impulse_a);
		node->K_impulse = PLAN_CONF_CHECK(impulse_k);

		node->Script = script;
		return node;
	}

	case Utiles::ENodeType::eFlyBy:
	{
		auto node = std::make_unique<Nodes::NodeFlyBy>();
		node->SphereRadius = PLAN_CONF_CHECK(sphereRarius);
		
		node->MismatchLimit = PLAN_CONF_CHECK(mismatchLimit);
		node->A_mismatch = PLAN_CONF_CHECK(mismatch_a);
		node->K_mismatch = PLAN_CONF_CHECK(mismatch_k);

		node->A_kink = PLAN_CONF_CHECK(kink_a);
		node->K_kink = PLAN_CONF_CHECK(kink_k);

		node->Script = script;
		return node;
	}
	default: throw std::runtime_error("Unexpected node type");
	}
}

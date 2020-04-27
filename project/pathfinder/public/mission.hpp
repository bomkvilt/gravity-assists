#ifndef PATHFINDER__MISSION_HPP
#define PATHFINDER__MISSION_HPP

#include "interfaces/ephemerides.hpp"


namespace Pathfinder
{
	struct NodeBase
	{
		using ptr = std::shared_ptr<NodeBase>;

		Ephemerides::IEphemerides::ptr Script;
		
		virtual bool Check(const FVector& Win, const FVector& Wout) const = 0;
	};

	struct NodeDeparture : public NodeBase
	{
		float EscapeSpeed = 0; // [m/s]
		
		bool Check(const FVector& Win, const FVector& Wout) const override;
	};

	struct NodeArrival : public NodeBase
	{
		float ParkingRadius  = 0; // [m]
		float SphereRadius   = 0; // [m]
		float ImpulseLimit_v = 0; // [m/s]

		bool Check(const FVector& Win, const FVector& Wout) const override;
	};

	struct NodeFlyBy : public NodeBase
	{
		float PlanetRadius = 0; // [m]
		float SphereRadius = 0; // [m]
		float MissmatchLimit_v = 0; // [m/s]

		bool Check(const FVector& Win, const FVector& Wout) const override;
	};
}

namespace Pathfinder
{
	struct Mission
	{
		std::vector<NodeBase::ptr> nodes;
		float normalFlyPeriodFactor = 0;
		float points_f0 = 0;
		float timeStep = 0;
		float timeTol = 0;
		float GM = 0;
		float t0 = 0;
	};
}


#endif //!PATHFINDER__MISSION_HPP

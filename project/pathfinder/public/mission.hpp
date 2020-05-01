#ifndef PATHFINDER__MISSION_HPP
#define PATHFINDER__MISSION_HPP

#include "interfaces/ephemerides.hpp"


namespace Pathfinder
{
	struct InNodeParams
	{
	public: // body relative velocities in solar system frame
		const FVector& W0; // [m/s] - departure
		const FVector& W1; // [m/s] - arrival
	};

	struct OutNodeParams
	{
	public:
		float Impulse  = 0; // [m/s] - impulse of speed the node inside
		float Mismatch = 0; // [m/s] - mismatch of in/out velocity
	};
}


namespace Pathfinder
{
	struct NodeBase
	{
		using ptr = std::shared_ptr<NodeBase>;

		Ephemerides::IEphemerides::ptr Script;
		
		virtual auto Check(const InNodeParams& in)->std::tuple<OutNodeParams, bool> const = 0;
	};

	struct NodeDepartureBase : public NodeBase
	{
		FReal ParkingRadius = 0; // [m]
		FReal SphereRadius  = 0; // [m]
		FReal ImpulseLimit  = 0; // [m/s]

		auto Check(const InNodeParams& in)->std::tuple<OutNodeParams, bool> const override;

		virtual FReal GetH0() const = 0;
	};

	struct NodeArrivalBase : public NodeBase
	{
		FReal ParkingRadius = 0; // [m]
		FReal SphereRadius  = 0; // [m]
		FReal ImpulseLimit  = 0; // [m/s]

		auto Check(const InNodeParams& in)->std::tuple<OutNodeParams, bool> const override;

		virtual FReal GetH1() const = 0;
	};

	struct NodeFlyBy : public NodeBase
	{
		FReal PlanetRadius  = 0; // [m]
		FReal SphereRadius  = 0; // [m]
		FReal MismatchLimit = 0; // [m/s]

		auto Check(const InNodeParams& in)->std::tuple<OutNodeParams, bool> const override;
	};
}


namespace Pathfinder::NodeDeparture
{
	struct EnergyDriven : public NodeDepartureBase
	{
		FReal h0;

		FReal GetH0() const override;
	};

	struct Circular : public NodeDepartureBase
	{
		FReal GetH0() const override;
	};
}


namespace Pathfinder::NodeArrival
{
	struct EnergyDriven : public NodeArrivalBase
	{
		FReal h1;

		FReal GetH1() const override;
	};

	struct Circular : public NodeArrivalBase
	{
		FReal GetH1() const override;
	};
}


namespace Pathfinder
{
	struct Mission
	{
		std::vector<NodeBase::ptr> nodes;
		FReal normalFlyPeriodFactor = 0;
		FReal points_f0 = 0;
		FReal timeStep = 0;
		FReal timeTol = 0;
		FReal GM = 0;
		FReal t0 = 0;
	};
}


#endif //!PATHFINDER__MISSION_HPP

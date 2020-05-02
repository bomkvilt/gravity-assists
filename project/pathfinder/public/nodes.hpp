#ifndef PATHFINDER__NODES_HPP
#define PATHFINDER__NODES_HPP

#include "mission.hpp"



namespace Pathfinder
{
	struct NodeDepartureBase : public NodeBase
	{
		FReal ParkingRadius = 0; // [m]
		FReal SphereRadius  = 0; // [m]
		FReal ImpulseLimit  = 0; // [m/s]
		FReal A_impulse     = 0;
		FReal K_impulse     = 0;

		auto Check(const InNodeParams& in, bool bGenCorrection)->std::tuple<OutNodeParams, bool> const override;

		virtual FReal GetH0() const = 0;
	};

	struct NodeArrivalBase : public NodeBase
	{
		FReal ParkingRadius = 0; // [m]
		FReal SphereRadius  = 0; // [m]
		FReal ImpulseLimit  = 0; // [m/s]
		FReal A_impulse     = 0;
		FReal K_impulse     = 0;

		auto Check(const InNodeParams& in, bool bGenCorrection)->std::tuple<OutNodeParams, bool> const override;

		virtual FReal GetH1() const = 0;
	};

	struct NodeFlyBy : public NodeBase
	{
		FReal PlanetRadius  = 0; // [m]
		FReal SphereRadius  = 0; // [m]
		FReal MismatchLimit = 0; // [m/s]
		FReal A_mismatch    = 0;
		FReal K_mismatch    = 0;
		FReal A_kink        = 0;
		FReal K_kink        = 0;

		auto Check(const InNodeParams& in, bool bGenCorrection)->std::tuple<OutNodeParams, bool> const override;
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


#endif //!PATHFINDER__NODES_HPP

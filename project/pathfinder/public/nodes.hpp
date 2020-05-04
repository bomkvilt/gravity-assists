#ifndef PATHFINDER__NODES_HPP
#define PATHFINDER__NODES_HPP

#include "mission.hpp"



namespace Pathfinder::Nodes
{
	struct NodeDepartureBase : public ScriptNode
	{
		FReal ParkingRadius = 0; // [m]
		FReal SphereRadius  = 0; // [m]
		FReal ImpulseLimit  = 0; // [m/s]
		FReal A_impulse     = 0;
		FReal K_impulse     = 0;

		auto Check(const InParams& in, bool bGenCorrection) const->std::tuple<OutParams, bool> override;

		virtual FReal GetH0() const = 0;
	};

	struct NodeArrivalBase : public ScriptNode
	{
		FReal ParkingRadius = 0; // [m]
		FReal SphereRadius  = 0; // [m]
		FReal ImpulseLimit  = 0; // [m/s]
		FReal A_impulse     = 0;
		FReal K_impulse     = 0;

		auto Check(const InParams& in, bool bGenCorrection) const->std::tuple<OutParams, bool> override;

		virtual FReal GetH1() const = 0;
	};

	struct NodeFlyBy : public ScriptNode
	{
		FReal PlanetRadius  = 0; // [m]
		FReal SphereRadius  = 0; // [m]
		FReal MismatchLimit = 0; // [m/s]
		FReal A_mismatch    = 0;
		FReal K_mismatch    = 0;
		FReal A_kink        = 0;
		FReal K_kink        = 0;

		auto Check(const InParams& in, bool bGenCorrection) const->std::tuple<OutParams, bool> override;
	};

	struct BurnNode : public Nodes::StaticNode
	{
		FReal ImpulseLimit = 0; // [m/s]
		FReal A_impulse    = 0;
		FReal K_impulse    = 0;

		auto Check(const InParams& in, bool bGenCorrection) const->std::tuple<OutParams, bool> override;
	};
}


namespace Pathfinder::NodeDeparture
{
	using Super = ::Pathfinder::Nodes::NodeDepartureBase;

	struct EnergyDriven : public Super
	{
		FReal h0;

		FReal GetH0() const override;
	};

	struct Circular : public Super
	{
		FReal GetH0() const override;
	};
}


namespace Pathfinder::NodeArrival
{
	using Super = ::Pathfinder::Nodes::NodeArrivalBase;

	struct EnergyDriven : public Super
	{
		FReal h1;

		FReal GetH1() const override;
	};

	struct Circular : public Super
	{
		FReal GetH1() const override;
	};
}


#endif //!PATHFINDER__NODES_HPP

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
		float Impulse    = 0; // [m/s] - impulse of speed the node inside
		float Mismatch   = 0; // [m/s] - mismatch of in/out velocity
		float Correction = 0; // [] - 
	};

	struct NodeBase
	{
		using ptr = std::shared_ptr<NodeBase>;

		Ephemerides::IEphemerides::ptr Script;

		virtual auto Check(const InNodeParams& in, bool bGenCorrection = false)->std::tuple<OutNodeParams, bool> const = 0;
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

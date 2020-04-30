#ifndef PATHFINDER__PATHFINDER_HPP
#define PATHFINDER__PATHFINDER_HPP

#include "mission.hpp"
#include "links.hpp"


namespace Pathfinder
{
	class PathFinder
	{
	public:
		struct FlightInfo
		{
			FReal totalMismatch = 0;
			FReal totalImpulse = 0;
			FReal totalTime = 0;
			FReal absTime = 0;
			Link::Link link;
		};

		struct FlightChain
		{
			std::vector<FlightInfo> chain;
			FReal totalMismatch = 0;
			FReal totalImpulse = 0;
			FReal totalTime = 0;

			FlightChain() = default;
			FlightChain(FlightChain&&) = default;
			FlightChain(const FlightChain&) = default;
			FlightChain(std::vector<FlightInfo>&& chain);
		};

	public:
		PathFinder(Mission&& mission);

		auto FirstApprox()->std::vector<FlightChain>;

	protected:
		Mission mission;
	};
}


#endif //!PATHFINDER__PATHFINDER_HPP

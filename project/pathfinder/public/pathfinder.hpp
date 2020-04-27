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
			float totalMismatch_v = 0;
			float totalImpulse = 0;
			float totalTime = 0;
			float absTime = 0;
			Link::Link link;
		};

	public:
		PathFinder(Mission&& mission);

		auto FirstApprox()->std::vector<std::vector<FlightInfo>>;

	protected:
		Mission mission;
	};
}


#endif //!PATHFINDER__PATHFINDER_HPP

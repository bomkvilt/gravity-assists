#ifndef PATHFINDER__UTILES_HPP
#define PATHFINDER__UTILES_HPP

#include "pathfinder.hpp"
#include "math/math.hpp"
#include "mission.hpp"
#include "links.hpp"



namespace Pathfinder::Solvers::Utiles
{
	FReal GetFlyTimeLimit(FReal r0, FReal r1, FReal factor, FReal GM);

	auto MakeRange(FReal min, FReal max, FReal steps)->std::vector<FReal>;

	void FindLinks(
		  std::vector<Link::Link>& links // found links
		, const Nodes::INode::ptr& A     // node to get out
		, const Nodes::INode::ptr& B     // node to get to
		, const MissionConfig& mission   // mission settings
		, const std::vector<FReal>& f0s  // toss angles
		, FReal t0                       // departure time
		, FReal GM						 // center body's gravity parameter
	);

	template<typename It, typename Fn>
	void FillTree(It bgn, It end, Fn clb)
	{
		It iA = bgn;
		It iB = bgn + 1;
		for (; iB != end; ++iA, ++iB)
		{
			clb(*iA, *iB, iB + 1 == end);
		}
	}

	template<typename C, typename Fn>
	void FillTree(C& container, Fn clb)
	{
		FillTree(container.begin(), container.end(), clb);
	}

	struct NodeA
	{
		const Nodes::INode::ptr& node;
		const std::vector<FReal>& f0s;
	};

	std::vector<PathFinder::FlightChain> ComputeFlight(
		  const MissionConfig& mission
		, const std::vector<NodeA>& nodes
		, FReal t0
		, FReal GM
		, bool bWithCorrection = false
	);
}


#endif //!PATHFINDER__UTILES_HPP

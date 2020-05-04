#ifndef PATHFINDER__SOLVER_UTILES_HPP
#define PATHFINDER__SOLVER_UTILES_HPP

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
		, const Mission& mission         // mission settings
		, const std::vector<FReal>& f0s  // toss angles
		, FReal t0                       // departure time
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
		Nodes::INode::ptr& node;
		std::vector<FReal>& f0s;
	};

	auto ComputeFlight(const std::vector<NodeA>& nodes, FReal t0_, const Mission& mission, bool bWithCorrection = false)->std::vector<PathFinder::FlightChain>;
}


#endif //!PATHFINDER__SOLVER_UTILES_HPP

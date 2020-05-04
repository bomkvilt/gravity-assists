#include "solvers/Utiles.hpp"
#include "solvers/pathTree.hpp"
#include "blocks/link.hpp"
#include <deque>



namespace Pathfinder::Solvers::Utiles
{
	FReal GetFlyTimeLimit(FReal r0, FReal r1, FReal factor, FReal GM)
	{
		const auto a = Math::Avg(r0, r1);
		return Math::Sqrt(a*a*a / GM) * 2 * Math::Pi * factor;
	}

	std::vector<FReal> MakeRange(FReal min, FReal max, FReal steps)
	{
		auto step  = (max - min) / steps;
		auto range = std::vector<FReal>();
		for (auto val = min; val < max && range.size() < steps; val += step)
		{
			range.push_back(val);
		}
		return range;
	}

	void FindLinks(
		  std::vector<Link::Link>& links
		, const Nodes::INode::ptr& A
		, const Nodes::INode::ptr& B
		, const Mission& mission
		, const std::vector<FReal>& f0s
		, FReal t0
	) {
		auto SetA_GM_t0 = [&A, &mission, t0](auto& conf)
		{
			auto [asScript, asStatic] = ::Pathfinder::Nodes::CastNode(A);
			conf.GM = mission.GM;
			conf.t0 = t0;
			if (asScript)
			{
				conf.SetA(asScript->Script);
			}
			else
			{
				conf.RA = asStatic->R;
			}
		};

		auto [asScript, asStatic] = Nodes::CastNode(B);
		if (asScript)
		{
			auto conf = Link::ScriptedLinkConfig();
			SetA_GM_t0(conf);
			conf.SetB(asScript->Script);
			conf.ts = mission.timeStep;
			conf.td = mission.timeTol / 100;
			conf.tt = mission.timeTol;
			conf.te = t0 + GetFlyTimeLimit(
				  conf.RA.Size()
				, conf.B.GetLocation().Size()
				, mission.normalFlyPeriodFactor
				, mission.GM
			);
			Link::FindLinks(links, conf, f0s);
		}
		else
		{
			auto conf = Link::StaticLinkConfig();
			SetA_GM_t0(conf);
			conf.RB = asStatic->R;
			Link::FindLinks(links, conf, f0s);
		}
	}

	auto ComputeFlight(const std::vector<NodeA>& nodes, FReal t0_, const Mission& mission, bool bWithCorrection) -> std::vector<PathFinder::FlightChain>
	{
		using Tree = PathTree<PathFinder::FlightInfo>;
		auto  tree = Tree();
		tree.RegisterOnAdded([](PathFinder::FlightInfo& parent, PathFinder::FlightInfo& child)
		{
			child.absTime += parent.absTime;
			child.totalTime += parent.totalTime;
			child.totalImpulse += parent.totalImpulse;
			child.totalMismatch += parent.totalMismatch;
		});
		auto rootID = tree.AppendPath([&]()->PathFinder::FlightInfo
		{
			auto node = PathFinder::FlightInfo();
			node.link.W1 = FVector(0, 0, 0);
			node.absTime = t0_;
			return node;
		}());
		
		auto parents  = std::deque<Tree::pathID>();
		auto children = std::deque<Tree::pathID>();
		parents.push_back(rootID);

		auto links = std::vector<Link::Link>();
		Utiles::FillTree(nodes, [&](const NodeA& iA, const NodeA& iB, bool bLast)
		{	// find all flights from A to B
			for (; parents.size(); parents.pop_front(), links.clear())
			{
				const auto  parentID = parents.front();
				const auto& parent = tree.GetPathByIF(parentID);
				
				// find all links from the departure time
				Utiles::FindLinks(links, iA.node, iB.node, mission, iA.f0s, parent.absTime);
				
				// create child nodes
				for (auto& link : links)
				{
					auto child = PathFinder::FlightInfo();
					{ // check out node
						auto params = Nodes::INode::InParams{ parent.link.W1, link.W0 };
						auto [res, bOK] = iA.node->Check(params, bWithCorrection);
						if (!bOK)
						{
							continue;
						}
						child.totalImpulse  += res.Impulse;
						child.totalMismatch += res.Mismatch;
					}
					if (bLast)
					{ // check in node
						auto params = Nodes::INode::InParams{ link.W1, FVector(0) };
						auto [res, bOK] = iB.node->Check(params, bWithCorrection);
						if (!bOK)
						{
							continue;
						}
						child.totalImpulse  += res.Impulse;
						child.totalMismatch += res.Mismatch;
					}
					child.link = link;
					child.absTime = link.dt;
					child.totalTime = link.dt;
					children.push_back(tree.AppendPath(child, parentID));
				}
			}
			std::swap(parents, children);
		});

		auto paths = std::vector<PathFinder::FlightChain>();
		for (auto& path : tree.GetFullPathByID(parents, true))
		{
			if (!path.size())
			{
				continue;
			}
			paths.emplace_back(std::move(path));
		}
		return paths;
	}
}

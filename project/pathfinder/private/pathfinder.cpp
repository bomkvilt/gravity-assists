#include "pathfinder.hpp"
#include "pathTree.hpp"
#include "link.hpp"
#include <deque>


namespace Pathfinder::utiles
{
	float GetFlyTimeLimit(
		  const Ephemerides::EphemeridesClient& A
		, const Ephemerides::EphemeridesClient& B
		, float factor
		, float GM
	) {
		using namespace Math;
		const auto r0 = A.GetLocation().Size();
		const auto r1 = B.GetLocation().Size();
		const auto a  = (r0 + r1) / 2;
		return Sqrt(a*a*a / GM) * 2 * Pi * factor;
	}

	std::vector<float> MakeRange(float min, float max, float steps)
	{
		auto step = (max - min) / steps;
		auto range = std::vector<float>();
		for (auto val = min; val < max; val += step)
		{
			range.push_back(val);
		}
		return range;
	}

	void FindLinks(
		  std::vector<Link::Link>& links
		, const Ephemerides::EphemeridesClient& A
		, const Ephemerides::EphemeridesClient& B
		, const std::vector<float>& f0s
		, float flyFactor
		, float timeStep
		, float timeTol
		, float t0
		, float GM
	) {
		auto conf = Link::FindLinksConfig();
		conf.A = A;
		conf.B = B;
		conf.GM = GM;
		conf.t0 = t0;
		conf.te = t0 + GetFlyTimeLimit(A, B, flyFactor, GM);
		conf.ts = timeStep;
		conf.tt = timeTol;
		Link::FindLinks(links, conf, f0s);
	}
}


namespace Pathfinder
{
	PathFinder::PathFinder(Mission&& inMission)
		: mission(std::move(inMission))
	{
		if (mission.nodes.size() < 2)
		{
			throw std::runtime_error("mission must consists from at least 2 nodes");
		}
		for (auto& node : mission.nodes)
		{
			if (!node)
			{
				throw std::runtime_error("mission nodes cannot be nulled");
			}
			if (!node->Script)
			{
				throw std::runtime_error("ephemeride connection must be defined");
			}
		}
	}

	auto PathFinder::FirstApprox()->std::vector<std::vector<FlightInfo>>
	{
		auto f0s = utiles::MakeRange(0, 2*Math::Pi, mission.points_f0);

		using Tree = PathTree<FlightInfo>;
		auto  tree = Tree();
		tree.RegisterOnAdded([](FlightInfo& parent, FlightInfo& child)
		{
			child.absTime += parent.absTime;
			child.totalTime += parent.totalTime;
			child.totalImpulse += parent.totalImpulse;
			child.totalMismatch_v += parent.totalMismatch_v;
		});
		auto rootID = tree.AppendPath([&]()->FlightInfo
		{
			auto node = FlightInfo();
			node.link.W1 = FVector(0, 0, 0);
			node.absTime = mission.t0;
			return node;
		}());
		
		auto parents  = std::deque<Tree::pathID>();
		auto children = std::deque<Tree::pathID>();
		parents.push_back(rootID);

		auto links = std::vector<Link::Link>();
		tree.FiilInByLayers(mission.nodes, [&](NodeBase::ptr& iA, NodeBase::ptr& iB, Tree::EFillInFlag flag)
		{	// find all flights from A to B
			for (; parents.size(); parents.pop_front(), links.clear())
			{
				const auto  parentID = parents.front();
				const auto& parent = tree.GetPathByIF(parentID);

				auto connA = Ephemerides::EphemeridesClient(iA->Script);
				auto connB = Ephemerides::EphemeridesClient(iB->Script);
				connA.SetTime(parent.absTime);
				connB.SetTime(parent.absTime);

				// find all links from the departure time
				utiles::FindLinks(links, connA, connB, f0s
					, mission.normalFlyPeriodFactor
					, mission.timeStep
					, mission.timeTol
					, mission.t0
					, mission.GM
				);
				// create child nodes
				for (auto& link : links)
				{
					if (!iA->Check(parent.link.W1, link.W0))
					{
						continue;
					}

					auto child = FlightInfo();
					child.link    = link;
					child.absTime = link.dt;
					if (flag & Tree::EFillInFlag::eFirstIteration)
					{
						child.totalImpulse += link.W0.Size();
					}
					else
					{
						child.totalMismatch_v += (child.link.W0 - parent.link.W1).Size();
					}
					if (flag & Tree::EFillInFlag::eLastIteration)
					{
						if (!iB->Check(link.W1, FVector()))
						{
							continue;
						}
						child.totalImpulse += link.W1.Size();
					}
					children.push_back(tree.AppendPath(child));
				}
			}
			std::swap(parents, children);
		});
		return tree.GetFullPathByID(parents);
	}
}

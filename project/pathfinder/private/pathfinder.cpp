#include "pathfinder.hpp"
#include "pathTree.hpp"
#include "link.hpp"
#include <deque>


namespace Pathfinder::utiles
{
	FReal GetFlyTimeLimit(
		  const Ephemerides::EphemeridesClient& A
		, const Ephemerides::EphemeridesClient& B
		, FReal factor
		, FReal GM
	) {
		using namespace Math;
		const auto r0 = A.GetLocation().Size();
		const auto r1 = B.GetLocation().Size();
		const auto a  = (r0 + r1) / 2;
		return Sqrt(a*a*a / GM) * 2 * Pi * factor;
	}

	std::vector<FReal> MakeRange(FReal min, FReal max, FReal steps)
	{
		auto step = (max - min) / steps;
		auto range = std::vector<FReal>();
		for (auto val = min; val < max && range.size() < steps; val += step)
		{
			range.push_back(val);
		}
		return range;
	}

	void FindLinks(
		  std::vector<Link::Link>& links
		, const Ephemerides::EphemeridesClient& A
		, const Ephemerides::EphemeridesClient& B
		, const std::vector<FReal>& f0s
		, FReal flyFactor
		, FReal timeStep
		, FReal timeTol
		, FReal t0
		, FReal GM
	) {
		auto conf = Link::FindLinksConfig();
		conf.A = A;
		conf.B = B;
		conf.GM = GM;
		conf.t0 = t0;
		conf.te = t0 + GetFlyTimeLimit(A, B, flyFactor, GM);
		conf.ts = timeStep;
		conf.tt = timeTol;
		conf.td = timeTol / 100;
		Link::FindLinks(links, conf, f0s);
	}

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

	auto PathFinder::FirstApprox()->std::vector<FlightChain>
	{
		auto f0s = utiles::MakeRange(0, 2*Math::Pi, mission.points_f0);

		using Tree = PathTree<FlightInfo>;
		auto  tree = Tree();
		tree.RegisterOnAdded([](FlightInfo& parent, FlightInfo& child)
		{
			child.absTime += parent.absTime;
			child.totalTime += parent.totalTime;
			child.totalImpulse += parent.totalImpulse;
			child.totalMismatch += parent.totalMismatch;
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
		utiles::FillTree(mission.nodes, [&](NodeBase::ptr& iA, NodeBase::ptr& iB, bool bLast)
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
					auto child = FlightInfo();
					{ // check out node
						auto params = InNodeParams{ parent.link.W1, link.W0 };
						auto [res, bOK] = iA->Check(params);
						if (!bOK)
						{
							continue;
						}
						child.totalImpulse  += res.Impulse;
						child.totalMismatch += res.Mismatch;
					}
					if (bLast)
					{ // check in node
						auto params = InNodeParams{ link.W1, FVector(0) };
						auto [res, bOK] = iB->Check(params);
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

		auto paths = std::vector<FlightChain>();
		for (auto& path : tree.GetFullPathByID(parents))
		{
			if (!path.size())
			{
				continue;
			}
			paths.emplace_back(std::move(path));
		}
		return paths;
	}

	PathFinder::FlightChain::FlightChain(std::vector<PathFinder::FlightInfo>&& chain_)
		: chain(std::move(chain_))
	{
		auto& last = chain.back();
		totalMismatch = last.totalMismatch;
		totalImpulse = last.totalImpulse;
		totalTime = last.totalTime;
	}
}

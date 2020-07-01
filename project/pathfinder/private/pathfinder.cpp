#include "solvers/FirstApprox.hpp"
#include "solvers/SecondApprox.hpp"



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
			if (!node->IsValid())
			{
				throw std::runtime_error("ephemeride connection must be defined");
			}
		}
	}

	auto PathFinder::FirstApprox(FReal timeOffset) -> const std::vector<FlightChain>&
	{
		auto t0 = mission.t0 + timeOffset;
		return firstApproxDB[t0] = Solvers::FirstApprox(mission, t0);
	}

	void PathFinder::SetFunctionality(Functionality functionality_)
	{
		functionality = functionality_;
	}

	auto PathFinder::GetFunctionalityBounds() const -> std::tuple<FReal, FReal>
	{
		if (!functionality)
		{
			throw std::runtime_error("functionality must be set to get functionality bounds");
		}

		auto min = FReal(NAN);
		auto max = FReal(NAN);
		for (const auto& pair : firstApproxDB)
		for (const auto& info : pair.second)
		{
			auto value = functionality(info);
			if (isnan(max) && isnan(min))
			{
				min = max = value;
				continue;
			}
			max = Math::Max(max, value);
			min = Math::Min(min, value);
		}
		return { min, max };
	}

	void PathFinder::FilterResults(std::function<void(const FirstApproxDB& db)> visiter)
	{
		if (!visiter)
		{
			throw std::runtime_error("visiter funtion must be defined");
		}
		visiter(firstApproxDB);
	}

	void PathFinder::FilterResults(FReal minFunctionalityToLeft)
	{
		if (!functionality)
		{
			throw std::runtime_error("functionality must be set to filter first approx trajectories");
		}

		auto pos1 = firstApproxDB.begin();
		while (pos1 != firstApproxDB.end())
		{
			auto& list = pos1->second;
			auto  pos2 = list.begin();
			while (pos2 != list.end())
			{
				auto value = functionality(*pos2);
				if (value > minFunctionalityToLeft)
				{
					pos2 = list.erase(pos2);
				}
				else ++pos2;
			}
			if (list.size() == 0)
			{
				pos1 = firstApproxDB.erase(pos1);
			}
			else ++pos1;
		}
	}

	const PathFinder::SecondApproxDB& PathFinder::SecondApprox()
	{
		if (!functionality)
		{
			throw std::runtime_error("functionality must be set for the operation");
		}

		for (auto& [t0, flights] : firstApproxDB)
		{
			for (auto& flight : flights)
			{
				SecondApprox(flight, t0);
			}
		}
		return secondApproxDB;
	}

	size_t PathFinder::FAXDBSize() const
	{
		size_t size = 0;
		for (auto& [_, data] : firstApproxDB)
		{
			size += data.size();
		}
		return size;
	}

	size_t PathFinder::SAXDBSize() const
	{
		return secondApproxDB.size();
	}

	const PathFinder::FirstApproxDB& PathFinder::GetFirstApproxDB() const
	{
		return firstApproxDB;
	}

	const PathFinder::SecondApproxDB& PathFinder::GetSecondApproxDB() const
	{
		return secondApproxDB;
	}

	void PathFinder::SecondApprox(const FlightChain& flight, Int64 t0)
	{
		auto [chain, value] = Solvers::SecondApprox(mission, flight, functionality);
		if (isnan(value))
		{
			return;
		}
		secondApproxDB.insert({ t0, {std::move(chain), value} });
	}
	
	PathFinder::FlightChain::FlightChain(std::vector<PathFinder::FlightInfo>&& chain_)
		: chain(std::move(chain_))
	{
		auto& last = chain.back();
		Correction = last.totalCorrection;
		Mismatch = last.totalMismatch;
		Impulse = last.totalImpulse;
		totalTime = last.totalTime;

		auto& first = chain.front();
		startTime = first.link.t0;
	}
}

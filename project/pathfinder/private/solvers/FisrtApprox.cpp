#include "solvers/FirstApprox.hpp"
#include "solvers/Utiles.hpp"



namespace Pathfinder::Solvers
{
	auto FirstApprox(const Mission& mission, FReal t0) -> std::vector<PathFinder::FlightChain>
	{
		auto f0s = Utiles::MakeRange(0, 2 * Math::Pi, mission.faxConfig.points_f0);
		auto seq = std::vector<Utiles::NodeA>();
		for (auto& node : mission.nodes)
		{
			seq.push_back({ node, f0s });
		}
		return ComputeFlight(mission.faxConfig, seq, t0, mission.GM);
	}
}

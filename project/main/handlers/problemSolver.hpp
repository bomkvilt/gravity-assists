#ifndef MAIN__PROBLEMSOLVER_HPP
#define MAIN__PROBLEMSOLVER_HPP

#include "configs/problemConfig.hpp"
#include <filesystem>



struct FlightDB : public reflect::FConfig
{
	ARCH_BEGIN(reflect::FConfig)
		ARCH_FIELD(, , flights)
		ARCH_END();
public:

	struct FlightRow : public Pathfinder::PathFinder::FlightChain
	{
		using Super = Pathfinder::PathFinder::FlightChain;

		ARCH_BEGIN(Super)
			ARCH_FIELD(, , functionality)
			ARCH_END();
	public:

		FlightRow() = default;

		FlightRow(const Super& rhs, FReal functionality = 0)
			: Super(rhs)
			, functionality(functionality)
		{}

		FReal functionality = 0;
	};

public:

	std::vector<FlightRow> flights;

public:

	void Save(const std::string& path)
	{
		if (!SaveConfig(path))
		{
			throw std::runtime_error("Cannot save floght db to destination file: '" + path + "'");
		}
	}
};



void SaveDB(const std::string& path, const Pathfinder::PathFinder::FirstApproxDB& db)
{
	auto data = FlightDB();
	for (auto& [_, list] : db)
	for (auto& flight : list )
	{
		data.flights.push_back(flight);
	}
	data.Save(path);
}

void SaveDB(const std::string& path, const Pathfinder::PathFinder::SecondApproxDB& db)
{
	auto data = FlightDB();
	for (auto& [_, flight] : db)
	{
		data.flights.push_back({ flight.chain, flight.functionality });
	}
	data.Save(path);
}



int SolveProblem(const std::string& path_)
{
	auto conf = ProblemConfig();
	if (!conf.LoadConfig(path_))
	{
		throw std::runtime_error("Cannot parse configuration file.");
	}

	auto path = std::filesystem::path(path_);
	auto dir  = path.root_directory();
	auto name = path.stem().wstring();
	auto faxPath = dir / (name + L".fax.json");
	auto saxPath = dir / (name + L".sax.json");

	auto solver = conf.MakeFinder();
	
	auto t0 = conf.timeSettings.t0;
	auto t1 = conf.timeSettings.t1;
	auto dt = conf.timeSettings.dt;
	for (auto t = t0; t <= t1; t += dt)
	{
		solver.FirstApprox(t);
	}

	auto [min, max] = solver.GetFunctionalityBounds();
	solver.FilterResults(min + (max - min) * conf.keepFactor);
	SaveDB(faxPath.string(), solver.GetFirstApproxDB());
	
	solver.SecondApprox();
	SaveDB(saxPath.string(), solver.GetSecondApproxDB());

	return 0;
}


#endif //!MAIN__PROBLEMSOLVER_HPP

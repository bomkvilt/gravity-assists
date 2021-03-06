#ifndef PATHFINDER__PATHFINDER_HPP
#define PATHFINDER__PATHFINDER_HPP

#include "mission.hpp"
#include "links.hpp"


namespace Pathfinder
{
	class PathFinder
	{
	public:
		struct FlightInfo : public reflect::FArchived
		{
			ARCH_BEGIN(reflect::FArchived)
				ARCH_FIELD(, , link)
				ARCH_FIELD(, , totalCorrection)
				ARCH_FIELD(, , totalMismatch)
				ARCH_FIELD(, , totalImpulse)
				ARCH_FIELD(, , totalTime)
				ARCH_FIELD(, , absTime)
				ARCH_END()
		public:
			FReal totalCorrection = 0;
			FReal totalMismatch = 0;
			FReal totalImpulse = 0;
			FReal totalTime = 0;
			FReal absTime = 0;
			Link::Link link;
		};

		struct FlightChain : public reflect::FArchived
		{
			ARCH_BEGIN(reflect::FArchived)
				ARCH_FIELD(, , chain)
				ARCH_FIELD(, , Correction)
				ARCH_FIELD(, , Mismatch)
				ARCH_FIELD(, , Impulse)
				ARCH_FIELD(, , totalTime)
				ARCH_FIELD(, , startTime)
				ARCH_END()
		public:
			std::vector<FlightInfo> chain;
			FReal Correction = 0;
			FReal Mismatch = 0;
			FReal Impulse = 0;
			FReal totalTime = 0;
			FReal startTime = 0;

			FlightChain() = default;
			FlightChain(std::vector<FlightInfo>&& chain);
		};
		
		struct SecondApproxData
		{
			FlightChain chain;
			FReal functionality = 0;
		};

		using FirstApproxDB  = std::map<Int64, std::vector<FlightChain>>;
		using SecondApproxDB = std::multimap<Int64, SecondApproxData>;
		using Functionality  = std::function<FReal(const FlightChain&)>;

	public:
		PathFinder(Mission&& mission);

		// creates a first approximation of flight trajectory
		auto FirstApprox(FReal timeOffset = 0)->const std::vector<FlightChain>&;

		// sets a functionality to map flight to one real value
		void SetFunctionality(Functionality functionality);

		// returns lower and upped bounds of functionality spectrum of all computed path
		auto GetFunctionalityBounds() const->std::tuple<FReal, FReal>;

		// modifies a DB with all paths of all computed time offsets
		void FilterResults(std::function<void(const FirstApproxDB& db)> visiter);
		void FilterResults(FReal minFunctionalityToLeft);

		// splits left first approx flights on two passive parts with a point with velocity impulce.
		// \note: count of links in SAX flight chain will be twice to the FAX's one
		const SecondApproxDB& SecondApprox();

		size_t FAXDBSize() const;
		size_t SAXDBSize() const;

		const FirstApproxDB& GetFirstApproxDB() const;
		const SecondApproxDB& GetSecondApproxDB() const;

	protected:
		void SecondApprox(const FlightChain& flight, Int64 t0);

	protected:
		Mission mission;
		
		Functionality functionality;
		FirstApproxDB firstApproxDB;
		SecondApproxDB secondApproxDB;
	};
}


#endif //!PATHFINDER__PATHFINDER_HPP

#include "trajectory/keplerOrbit.hpp"
#include "solvers/SecondApprox.hpp"
#include "solvers/Utiles.hpp"
#include "defer.hpp"

#include <gsl/gsl_multimin.h>
#include <gsl/gsl_errno.h>



namespace Pathfinder::Solvers::Utiles
{
	auto GetBurnParams(const Link::Link& link, FReal dQfactor) -> std::tuple<FVector, FReal>
	{
		auto q = link.GetRealAnomaly(dQfactor);
		auto R = link.GetTragectoryPoint(q);
		auto f = link.GetTossAngle(q);
		auto Q = q - link.w;
		return { R, f - Q };
	}


	class StateVectorMap final
	{
		std::vector<FReal*> data;
		std::vector<FReal> steps;

	public:

		void SetSize(int n)
		{
			data .resize(n, nullptr);
			steps.resize(n, NAN);
		}

		FReal& Assign(int n, FReal& field)
		{
			assert(n < data.size());
			data[n] = &field;
			return field;
		}

		void SetStep(int n, FReal value)
		{
			assert(n < steps.size());
			steps[n] = value;
		}

		void Sync_x0_ss(gsl_vector* x0, gsl_vector* ss)
		{
			assert(x0); assert(x0->size == data.size());
			assert(ss); assert(ss->size == data.size());
			for (auto i = 0; i < data.size(); ++i)
			{
				assert(data[i]);
				assert(!isnan(steps[i]));
				gsl_vector_set(x0, i, *data[i]);
				gsl_vector_set(ss, i, steps[i]);
			}
		}

		void ReadVector(const gsl_vector* v)
		{
			assert(v); assert(v->size == data.size());
			for (auto i = 0; i < data.size(); ++i)
			{
				auto field = data[i];
				assert(field);

				*data[i] = gsl_vector_get(v, i);
			}
		}
	};


	struct SecondApproxHelper
	{
		using TossAgles = std::vector<std::vector<FReal>>;
		using BurnNodes = std::vector<Nodes::INode::ptr>;
		using Chain     = std::vector<Utiles::NodeA>;
		using Mapper      = PathFinder::Functionality;
		using FlightChain = PathFinder::FlightChain;

		using Functor       = gsl_multimin_function;
		using GSL_vector    = gsl_vector*;
		using GSL_minimiser = gsl_multimin_fminimizer*;

	public:
		int k = 0; // number of flights
		int m = 0; // number of variables
		
		// << gsn entery
		Functor       fr;
		GSL_vector    x0 = nullptr;
		GSL_vector    ss = nullptr;
		GSL_minimiser mz = nullptr;

		FReal t  = 0;
		FReal GM = 0;

		Chain chain;
		Mapper functionality;
		TossAgles tossAngles;
		BurnNodes burnNodes;
		StateVectorMap fieldMap;
		
		FReal curFunctionality = NAN;
		FlightChain currentFlight;

		const SAXConfig& mission;

	public:
		SecondApproxHelper(const Mission& mission, const PathFinder::FlightChain& flight, const PathFinder::Functionality& functionality)
			: mission(mission.saxConfig)
			, k(flight.chain.size())
			, m(flight.chain.size()*5 + 1)
			, functionality(functionality)
			, GM(mission.GM)
		{
			// create a list of toss angles
			// \note: born nodes are included too
			for (auto i = 0; i < m; ++i)
			{
				tossAngles.push_back({ 0 });
			}

			// create vectors of states and step sizes
			x0 = gsl_vector_alloc(m);
			ss = gsl_vector_alloc(m);

			// create extended mission chain
			burnNodes.reserve(k);
			fieldMap .SetSize(m);
			ParseFlight(flight, mission.nodes);

			// create a functor
			fr.params = this;
			fr.n = m;
			fr.f = [](const gsl_vector* v, void* params)->double
			{
				auto self = (SecondApproxHelper*)params;
				return self->ComputeFunctionality(v);
			};
		}

		~SecondApproxHelper()
		{
			if (x0) gsl_vector_free(x0);
			if (ss) gsl_vector_free(ss);
			if (mz) gsl_multimin_fminimizer_free(mz);
		}

		void ParseFlight(const PathFinder::FlightChain& flight, const Mission::Nodes& nodes)
		{
			auto fpos = flight.chain.begin();
			auto cpos = nodes.begin();
			auto cend = nodes.end();
			for (size_t i = 0; cpos != cend; ++cpos, ++i)
			{
				auto& node = *cpos;
				auto bLast = cpos + 1 == cend;
				auto fieldOffset = 5 * i;
				auto chainOffset = 2 * i;

				chain.push_back({ *cpos, tossAngles[chainOffset + 0] });

				if (bLast) continue;
				
				if (auto burn = mission.burnNodeFactory())
				{
					burnNodes.push_back(burn);

					auto [R, f] = Utiles::GetBurnParams(fpos->link, mission.burnArcFraction);
					fieldMap.Assign(fieldOffset + 0, tossAngles[chainOffset + 0][0]) = fpos->link.f0;
					fieldMap.Assign(fieldOffset + 1, tossAngles[chainOffset + 1][0]) = f;
					fieldMap.Assign(fieldOffset + 2, burn->R.x) = R.x;
					fieldMap.Assign(fieldOffset + 3, burn->R.y) = R.y;
					fieldMap.Assign(fieldOffset + 4, burn->R.z) = R.z;

					fieldMap.SetStep(fieldOffset + 0, mission.initialTossAngleStep);
					fieldMap.SetStep(fieldOffset + 1, mission.initialTossAngleStep);
					fieldMap.SetStep(fieldOffset + 2, mission.initialBurnPointStep);
					fieldMap.SetStep(fieldOffset + 3, mission.initialBurnPointStep);
					fieldMap.SetStep(fieldOffset + 4, mission.initialBurnPointStep);
				
					chain.push_back(NodeA{ burnNodes.back(), tossAngles[chainOffset + 1] });

					++fpos;
				}
				else throw std::runtime_error("burn node cannot be nullptr");
			}
			fieldMap.Assign(m - 1, t) = flight.startTime;
			fieldMap.SetStep(m - 1, mission.initialTimeStep);
			fieldMap.Sync_x0_ss(x0, ss);
		}

		bool InitMinimiser()
		{
			if (mz)	return true;

			gsl_set_error_handler_off();

			auto T = gsl_multimin_fminimizer_nmsimplex2;
			mz = gsl_multimin_fminimizer_alloc(T, m);
			
			auto status = gsl_multimin_fminimizer_set(mz, &fr, x0, ss);
			if (status == GSL_SUCCESS || status == GSL_EBADFUNC)
			{
				return !status;
			}
			throw std::runtime_error("Unexpected status from minimiser initialisation: " + std::to_string(status));
		}

	public:

		double ComputeFunctionality(const gsl_vector* v)
		{
			assert(functionality);
			fieldMap.ReadVector(v);
			auto results = ComputeFlight(mission, chain, t, GM, true);
			if (!results.size())
			{
				return NAN;
			}
			
			int i_min = 0;
			FReal min = NAN;
			for (auto i = 0; i < results.size(); ++i)
			{
				auto val = functionality(results[i]);
				if (val < min || isnan(min))
				{
					i_min = i;
					min = val;
				}
			}
			curFunctionality = min;
			std::swap(currentFlight, results[i_min]);

			return min;
		}

		bool FindMinimum()
		{
			if (!InitMinimiser())
			{
				return false;
			}

			auto status = int(GSL_CONTINUE);
			auto prevValue = FReal(NAN);
			auto min_delta = mission.minMinimisationDelta;
			auto max_iter = mission.maxMinimisationIters;
			for (int iter = 0; status == GSL_CONTINUE && iter < max_iter; ++iter)
			{
				if (status = gsl_multimin_fminimizer_iterate(mz))
				{
					return false;
				}

				FReal curValue = mz->fval;
				if (!isnan(prevValue))
				{
					auto delta = Math::Abs(curValue - prevValue);
					status = delta < min_delta ? GSL_SUCCESS : GSL_CONTINUE;
				}
				else
				{
					status = GSL_CONTINUE;
				}
				prevValue = curValue;
			}
			ComputeFunctionality(mz->x);
			return true;
		}
	};
}


namespace Pathfinder::Solvers
{
	std::tuple<PathFinder::FlightChain, FReal> SecondApprox(
		  const Mission& mission
		, const PathFinder::FlightChain& flight
		, const PathFinder::Functionality& functionality
		// , FReal tMin
		// , FReal tMax
	) {
		auto helper = Utiles::SecondApproxHelper(mission, flight, functionality);
		if (!helper.FindMinimum() && isnan(helper.curFunctionality))
		{
			return { {}, NAN };
		}
		return { helper.currentFlight, helper.curFunctionality };
	}
}

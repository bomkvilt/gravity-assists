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
		auto Q = link.Q0 + (link.Q1 - link.Q0) * dQfactor;
		auto q = link.w  + Q;
		auto r = Kepler::r(link.p, link.e, q);
		auto A = FQuat((link.R0 ^ link.R1).GetNormal(), RAD2DEG(q) * (link.bf ? 1 : -1));
		auto R =  r * (A * link.R0.GetNormal());
		auto f = Kepler::Elliptic::f(Q, q, link.e, link.bf);
		return { R, f };
	}


	class StateVectorMap final
	{
		std::vector<FReal*> data;

	public:

		void SetSize(int n)
		{
			data.resize(n, nullptr);
		}

		FReal& Assign(int n, FReal& field)
		{
			assert(n < data.size());
			data[n] = &field;
			return field;
		}

		void Sync_x0_ss(gsl_vector* x0, gsl_vector* ss, FReal factor)
		{
			assert(x0); assert(x0->size == data.size());
			assert(ss); assert(ss->size == data.size());
			for (auto i = 0; i < data.size(); ++i)
			{
				auto field = data[i];
				assert(field);

				auto val = *field;
				gsl_vector_set(x0, i, val);
				gsl_vector_set(ss, i, val * factor);
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
		using Chain     = std::vector<Utiles::NodeA>;
		using BurnNodes = std::vector<Nodes::StaticNode::ptr>;
		using Mapper    = PathFinder::Functionality;

		using Functor       = gsl_multimin_function;
		using GSL_vector    = gsl_vector*;
		using GSL_minimiser = gsl_multimin_fminimizer*;

	public:
		int k = 0; // number of flights
		int m = 0; // number of variables
		
		// << gsn entery
		Functor fr; // functor to get minimised
		GSL_vector x0 = nullptr; // initial values
		GSL_vector ss = nullptr; // step sizes
		GSL_minimiser mz = nullptr; // minimiser

		FReal t = 0;
		Chain chain;
		Mapper functionality;
		TossAgles tossAngles;
		BurnNodes burnNodes;
		StateVectorMap fieldMap;
		
		FReal curFunctionality = NAN;
		PathFinder::FlightChain currentFlight;

		Mission& mission;

	public:
		SecondApproxHelper(Mission& mission, const PathFinder::FlightChain& flight, PathFinder::Functionality functionality)
			: mission(mission)
			, k(flight.chain.size())
			, m(flight.chain.size() * 5 + 1)
			, functionality(functionality)
		{
			// create a list of toss angles
			// \note: born nodes are included too
			for (auto i = 0; i < 2*k + 1; ++i)
			{
				tossAngles.push_back({ 0 });
			}

			// create vectors of states and step sizes
			auto x0 = gsl_vector_alloc(m);
			auto ss = gsl_vector_alloc(m);

			// create extended mission chain
			fieldMap.SetSize(m);
			ParseFlight(flight);

			// create a functor
			fr.params = this;
			fr.n = m;
			fr.f = [](const gsl_vector* v, void* params)->double
			{
				auto self = (SecondApproxHelper*)params;
				self->fieldMap.ReadVector(v);
				return self->ComputeFunctionality();
			};

			// create a minimiser
			auto T = gsl_multimin_fminimizer_nmsimplex2;
			mz = gsl_multimin_fminimizer_alloc(T, m);
		}

		~SecondApproxHelper()
		{
			if (x0) gsl_vector_free(x0);
			if (ss) gsl_vector_free(ss);
			if (mz) gsl_multimin_fminimizer_free(mz);
		}

		void ParseFlight(const PathFinder::FlightChain& flight)
		{
			auto fpos = flight .chain.begin();
			auto cpos = mission.nodes.begin();
			auto cend = mission.nodes.end();
			for (auto i = 0; cpos != cend; ++cpos, i += 2)
			{
				auto& node = *cpos;
				auto bLast = cpos + 1 == cend;

				chain.push_back({ *cpos, tossAngles[i] });

				if (bLast) continue;
				
				if (auto burn = mission.burnNodeFactory())
				{
					burnNodes.push_back(burn);

					auto [R, f] = Utiles::GetBurnParams((fpos++)->link, mission.burnArcFraction);
					fieldMap.Assign(i + 0, tossAngles[i + 0][0]) = fpos->link.f0;
					fieldMap.Assign(i + 1, tossAngles[i + 1][0]) = f;
					fieldMap.Assign(i + 2, burn->R.x) = R.x;
					fieldMap.Assign(i + 3, burn->R.y) = R.y;
					fieldMap.Assign(i + 4, burn->R.z) = R.z;
				
					chain.push_back({ std::dynamic_pointer_cast<Nodes::INode>(burn), tossAngles[i + 1] });
				}
				else throw std::runtime_error("burn node cannot be nullptr");
			}
			fieldMap.Assign(m - 1, t) = flight.startTime;
			fieldMap.Sync_x0_ss(x0, ss, 0.001);
		}

		void InitMinimiser()
		{
			if (!mz)
			{
				return;
			}

			gsl_set_error_handler_off();

			if (auto code = gsl_multimin_fminimizer_set(mz, &fr, x0, ss))
			{
				throw std::runtime_error("unexpected code during minimiser initialisation: " + std::to_string(code));
			}				
		}

	public:

		double ComputeFunctionality()
		{
			assert(functionality);
			auto results = ComputeFlight(chain, t, mission, true);
			
			int i_min = 0;
			FReal min = NAN;
			for (auto i = 0; i < results.size(); ++i)
			{
				auto val = functionality(results[i]);
				if (val < min)
				{
					i_min = i;
					min = val;
				}
			}
			std::swap(currentFlight, results[i_min]);

			return min;
		}

		bool FindMinimum()
		{
			InitMinimiser();

			FReal prevValue = NAN;
			FReal min_delta = mission.minMinimisationDelta;
			int status = GSL_CONTINUE;
			int max_iter = mission.maxMinimisationIters;
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
				curFunctionality = curValue;
				prevValue = curValue;
			}
			return true;
		}
	};
}


namespace Pathfinder::Solvers
{
	auto SecondApprox(
		  Mission& mission
		, const PathFinder::FlightChain& flight
		, PathFinder::Functionality functionality
	)->std::tuple<PathFinder::FlightChain, FReal>
	{
		auto helper = Utiles::SecondApproxHelper(mission, flight, functionality);
		if (!helper.FindMinimum())
		{
			return { {}, NAN };
		}
		return { helper.currentFlight, helper.curFunctionality };
	}
}

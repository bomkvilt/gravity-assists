#include "link.hpp"
#include "defer.hpp"
#include "keplerOrbit.hpp"
#include <gsl/gsl_errno.h>
#include <gsl/gsl_multimin.h>
#include <array>


namespace Pathfinder::Link::Utiles
{
	namespace kepel = ::Pathfinder::Kepler::Elliptic;

	LinkAdapter::LinkAdapter(const FindLinksConfig& cfg, FReal f0)
		: A(cfg.A)
		, B(cfg.B)
		, GM(cfg.GM)
	{
		this->A.SetTime(cfg.t0);
		this->t0 = t0;
		this->f0 = kepel::NZ(f0);
		this->bf = kepel::bf(Q0, f0);
		this->R0 = A.GetLocation();
		this->r0 = R0.Size();
	}

	bool LinkAdapter::Find_t(FReal t)
	{
		B.SetTime(t);
		R1 = B.GetLocation();
		r1 = R1.Size();
		Q1 = Q0 + Math::Angle2(R0, R1, Math::EPosAngles());

		auto [res, bOK] = kepel::epwqq(r0, r1, Q0, Q1, f0);
		if (bOK)
		{
			e = res.e;
			p = res.p;
			w = res.w;
			q0 = res.q0;
			q1 = res.q1;

			a  = kepel::a(e, p);
			E0 = kepel::E(q0, r0, e);
			E1 = kepel::E(q1, r1, e);
			M0 = kepel::M(E0, e);
			M1 = kepel::M(E1, e);
			dt = kepel::dt(M0, M1, a, GM, bf);
			if (isinf(dt) || isnan(dt))
			{
				return false;
			}

			t1 = t0 + dt;
		}
		return bOK;
	}

	void LinkAdapter::FixParams()
	{
		Fix2DParams();
		Fix3DParams();
	}

	void LinkAdapter::Fix2DParams()
	{
		v0 = kepel::v(q0, e, p, GM);
		v1 = kepel::v(q1, e, p, GM);
		f1 = kepel::f(Q1, q1, e, bf);
	}

	void LinkAdapter::Fix3DParams()
	{
		// >> gloabal basis
		auto x = FVector(1, 0, 0);
		auto z = FVector(0, 0, 1);
		// >> local plane's basis
		auto X = (R0     ).GetNormal();
		auto Z = (R0 ^ R1).GetNormal();
		// >> rotation xyz -> XYZ
		auto rotP = Math::BasisTranslation(x, z, X, Z);
		// velocity xyz orientations
		auto rot0 = rotP * FQuat(FVector(0, 0, 1), RAD2DEG(f0));
		auto rot1 = rotP * FQuat(FVector(0, 0, 1), RAD2DEG(f1));
		// >> velocities in XYZ
		V0 = rot0 * FVector(v0, 0, 0);
		V1 = rot1 * FVector(v1, 0, 0);
		// >> planet relative velocities
		W0 = V0 - A.GetVelocity();
		W1 = V1 - B.GetVelocity();
	}
}


namespace Pathfinder::Link::Utiles
{
	class RootWindowHelper final
	{
	public:
		enum EState
		{
			  ePositive = 1 << 0
			, eNegative = 1 << 1
			, eZero		= 1 << 2
			, eNAN		= 1 << 3
		};

		enum class EPatternType
		{
			  eNone
			, eRoot
			, eSign
			, eExtr
		};

		struct Point
		{
			EState state = EState::eNAN;
			FReal  delta = 0;
			FReal  time = 0;
		};

	private:
		std::array<Point, 3> window;
		FReal DTOL;

	public:
		RootWindowHelper(FReal DTOL = Math::Epsilon)
			: DTOL(DTOL)
		{}

		void Push(FReal time, FReal delta)
		{
			window[2] = window[1];
			window[1] = window[0];
			window[0].state = DeduceState(delta);
			window[0].delta = delta;
			window[0].time = time;
		}

		EState DeduceState(FReal delta)
		{
			return isnan(delta)               ? EState::eNAN 
				: Math::Equal(delta, 0, DTOL) ? EState::eZero
				: delta > 0                   ? EState::ePositive : EState::eNegative
				;
		}

		auto GetRoot()->std::tuple<Point, Point, EPatternType>
		{
			auto pattern = GetPattern();
			switch (pattern)
			{
			case EPatternType::eRoot: {
				auto p = window[0];
				return { p, p, pattern };
			}
			case EPatternType::eSign: {
				auto p0 = window[0];
				auto p1 = window[1];
				return { p1, p0, pattern };
			}
			case EPatternType::eExtr: {
				auto p0 = window[0];
				auto p1 = window[2];
				return { p1, p0, pattern };
			}
			case EPatternType::eNone:
				return { {}, {}, pattern };
			}
			throw std::runtime_error("unexpected pattern type: (" + std::to_string((int)pattern) + ")");
		}

		bool CheckRoot()
		{
			return GetPattern() != EPatternType::eNone;
		}

		EPatternType GetPattern()
		{
			auto s0 = window[0].state;
			auto s1 = window[1].state;
			auto s2 = window[1].state;
			if (s0 == EState::eZero)
			{
				return EPatternType::eRoot;
			}
			if ((s0 | s1) == (EState::eNegative | EState::ePositive))
			{
				return EPatternType::eSign;
			}
			if ((s0 & s1 & s2) & (EState::eNegative | EState::ePositive)) 
			{
				using namespace Math;
				const auto v3 = Abs(window[0].delta); // n
				const auto v2 = Abs(window[1].delta); // n - 1
				const auto v1 = Abs(window[2].delta); // n - 2
				const auto d12 = v2 - v1;
				const auto d13 = v3 - v1;
				if (Equal(d13, 0) && d12 < 0)
				{
					return EPatternType::eExtr;
				}
				if (d13 > 0 && d12 < d13 / 4)
				{
					return EPatternType::eExtr;
				}
			}
			return EPatternType::eNone;
		}
	};
}


namespace Pathfinder::Link::Utiles
{
	bool FindAsRoot(LinkAdapter& link, FReal t0, FReal t1, FReal v0, FReal v1, FReal DTOL, FReal TTOL)
	{
		using namespace Math;
		do
		{
			auto tm = (t0 + t1) / 2;
			if (!link.Find_t(tm))
			{
				return false;
			}

			auto vm = tm - link.t1;
			if (Equal(vm, 0, DTOL))
			{
				return true;
			}
			
			if (Sign(v0) == Sign(vm))
			{ 
				t0 = tm; v0 = vm; 
			}
			else 
			{ 
				t1 = tm; v1 = vm; 
			}
		} while (Abs(t1 - t0) > TTOL);

		return false;
	}

	bool FindMinimum(LinkAdapter& link, FReal t0, FReal t1, FReal v0, FReal v1, FReal DTOL, FReal TTOL)
	{
		// mirror the problem to positive space
		v0 = Math::Abs(v0);
		v1 = Math::Abs(v1);

		// function to minimize
		auto F = gsl_multimin_function();
		F.params = &link; 
		F.n = 1;
		F.f = [](const gsl_vector* v, void* params)->double
		{
			auto t = gsl_vector_get(v, 0);
			auto link = (LinkAdapter*)params;
			if (!link->Find_t(t))
			{ return NAN; }
			return Math::Abs(t - link->t1);
		};

		// initial point and stepsize
		auto x  = gsl_vector_alloc(1); // starting point
		auto ss = gsl_vector_alloc(1); // initial step sizes
		DEFER(_v)[x, ss]()
		{
			gsl_vector_free(x);
			gsl_vector_free(ss);
		};
		gsl_vector_set(ss, 0, (t1-t0)/50);
		gsl_vector_set(x , 0, Math::Avg(t0, t1));

		// create minimizer
		auto T = gsl_multimin_fminimizer_nmsimplex2;
		auto s = gsl_multimin_fminimizer_alloc(T, 1);
		DEFER(_)[s]()
		{
			gsl_multimin_fminimizer_free(s);
		};

		constexpr auto check = [](int status)
		{
			if (status != GSL_SUCCESS && status != GSL_EBADFUNC)
			{
				throw std::runtime_error("unexpected state from multimin: " + std::to_string(status));
			}
		};

		// initialize the minimizer
		check(gsl_multimin_fminimizer_set(s, &F, x, ss));

		// find minimum
		int status = GSL_CONTINUE;
		for (int iter = 0, max_iter = 100; status == GSL_CONTINUE && iter < max_iter; ++iter)
		{
			auto status = gsl_multimin_fminimizer_iterate(s);
			if (status == GSL_EBADFUNC)
			{
				return false;
			}
			else check(status);
			
			auto tm = gsl_vector_get(s->x, 0);
			if (!(tm >= t0 && tm <= t1))
			{
				return false;
			}

			if (s->fval <= DTOL)
			{
				return true;
			}

			auto size = gsl_multimin_fminimizer_size(s);
			status = gsl_multimin_test_size(size, TTOL);
		}
		return false;
	}
}


namespace Pathfinder::Link
{
	auto FindLinks(std::vector<Link>& links, const FindLinksConfig& cfg, FReal f0)
	{
		using namespace Math;
		auto link = Utiles::LinkAdapter(cfg, f0);

		// find roots of (t_expected - t_required) function
		auto window = Utiles::RootWindowHelper(cfg.ts / 10);
		for (auto t_exp = cfg.t0; t_exp < cfg.te; t_exp += cfg.ts)
		{
			if (!link.Find_t(t_exp))
			{
				window.Push(t_exp, NAN);
				continue;
			}

			window.Push(t_exp, t_exp - link.t1);
			if (!window.CheckRoot())
			{
				continue;
			}

			auto [p0, p1, mode] = window.GetRoot();
			switch (mode) 
			{
			case Utiles::RootWindowHelper::EPatternType::eRoot: break;
			case Utiles::RootWindowHelper::EPatternType::eSign:
				if (!Utiles::FindAsRoot(link, p0.time, p1.time, p0.delta, p1.delta, cfg.tt, cfg.td))
				{
					continue;
				} 
				else break;

			case Utiles::RootWindowHelper::EPatternType::eExtr:
				if (!Utiles::FindMinimum(link, p0.time, p1.time, p0.delta, p1.delta, cfg.tt, cfg.td))
				{
					continue;
				}
				else break;

			default: throw std::runtime_error("unexpected mode: " + std::to_string((int)mode));
			}
			link.FixParams();
			links.push_back(link);
		}
	}

	void FindLinks(std::vector<Link>& links, const FindLinksConfig& cfg, const std::vector<FReal>& f0s)
	{
		for (auto f0 : f0s)
		{
			FindLinks(links, cfg, f0);
		}
	}
}

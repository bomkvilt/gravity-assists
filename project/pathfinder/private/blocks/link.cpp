#include "blocks/link.hpp"
#include "trajectory/keplerOrbit.hpp"
#include "defer.hpp"

#include <gsl/gsl_errno.h>
#include <gsl/gsl_multimin.h>
#include <array>



namespace Pathfinder::Link
{
	void ChechValue(FReal val)
	{
		if (isnan(val))
		{
			throw std::runtime_error("value cannot be NAN");
		}
	}

	void ScriptedLinkConfig::SetA(Ephemerides::IEphemerides& script)
	{
		ChechValue(t0);
		auto [RA_, VA_] = script.GetMovement(t0);
		RA = RA_;
		VA = VA_;
	}

	void ScriptedLinkConfig::SetB(Ephemerides::IEphemerides& script)
	{
		ChechValue(t0);
		B.SetDriver(&script);
		B.SetTime(t0);
	}

	void ScriptedLinkConfig::SetA(Ephemerides::IEphemerides::ptr& script)
	{
		SetA(*script);
	}

	void ScriptedLinkConfig::SetB(Ephemerides::IEphemerides::ptr& script)
	{
		SetB(*script);
	}


	void StaticLinkConfig::SetA(Ephemerides::IEphemerides& script)
	{
		ChechValue(t0);
		auto [RA_, VA_] = script.GetMovement(t0);
		RA = RA_;
		VA = VA_;
	}

	void StaticLinkConfig::SetA(Ephemerides::IEphemerides::ptr& script)
	{
		SetA(*script);
	}
}


namespace Pathfinder::Link
{
	FVector Link::GetAxisX() const
	{
		return R0.GetNormal();
	}

	FVector Link::GetAxisY() const
	{
		return -(GetAxisX() ^ GetAxisZ());
	}

	FVector Link::GetAxisZ() const
	{
		return (R0 ^ R1).GetNormal() * (Q1 - Q0 <= Math::Pi ? 1 : -1);
	}

	FVector Link::GetTragectoryPoint(FReal q) const
	{
		auto r = Kepler::r(p, e, q);
		auto A = FQuat(GetAxisZ(), RAD2DEG(q - q0));
		return r * (A * GetAxisX());
	}

	FReal Link::GetRealAnomaly(FReal fraction) const
	{
		assert(fraction >= 0 && fraction <= 1);
		return bf
			? q0 + fraction*(q1 - q0)
			: q1 + fraction*(q0 - q1 + 2*Math::Pi)
			;
	}

	FReal Link::GetTossAngle(FReal q) const
	{
		return Kepler::Elliptic::f(q - w, q, e, bf);
	}
}


namespace Pathfinder::Link::Utiles
{
	namespace kepel = ::Pathfinder::Kepler::Elliptic;

	LinkAdapter::LinkAdapter(FReal GM)
		: GM(GM)
	{}

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
		// velocity orientations in XYZ
		auto rot0 = FQuat({ 0, 0, 1 }, RAD2DEG(f0));
		auto rot1 = FQuat({ 0, 0, 1 }, RAD2DEG(f1));
		// >> velocities in XYZ
		V0 = rot0 * FVector(v0, 0, 0);
		V1 = rot1 * FVector(v1, 0, 0);

		// >> gloabal basis
		auto x = FVector(1, 0, 0);
		auto z = FVector(0, 0, 1);
		// >> local plane's basis
		auto X = GetAxisX();
		auto Z = GetAxisZ();
		// >> rotation xyz -> XYZ
		auto rotP = Math::BasisTranslation(x, z, X, Z);
		// >> velocity in xyz
		V0 = rotP * V0;
		V1 = rotP * V1;
		
		// >> planet relative velocities
		FixW01();
	}

	bool LinkAdapter::Find_t()
	{
		if (auto [res, bOK] = kepel::epwqq(r0, r1, Q0, Q1, f0); bOK)
		{
			e  = res.e;
			p  = res.p;
			w  = res.w;
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
			
			return true;
		}
		return false;
	}


	ScriptedLink::ScriptedLink(const ScriptedLinkConfig& conf, FReal f0_)
		: LinkAdapter(conf.GM)
		, VA(conf.VA)
		, B (conf.B )
	{
		t0 = conf.t0;
		R0 = conf.RA;
		r0 = R0.Size();
		f0 = kepel::NZ(f0_);
		bf = kepel::bf(Q0, f0);
	}
	
	void ScriptedLink::FixW01()
	{
		W0 = V0 - VA;
		W1 = V1 - B.GetVelocity();
	}
	
	bool ScriptedLink::Find_t(FReal t)
	{
		B.SetTime(t);
		R1 = B.GetLocation();
		r1 = R1.Size();
		Q1 = Q0 + Math::Angle2(R0, R1, Math::EPosAngles());
		return LinkAdapter::Find_t();
	}


	StaticLink::StaticLink(const StaticLinkConfig& conf, FReal f0_)
		: LinkAdapter(conf.GM)
		, VA(conf.VA)
		, VB(conf.VB)
	{
		R0 = conf.RA;
		R1 = conf.RB;
		t0 = conf.t0;
		Q1 = Q0 + Math::Angle2(R0, R1, Math::EPosAngles());
		r0 = R0.Size();
		r1 = R1.Size();
		f0 = kepel::NZ(f0_);
		bf = kepel::bf(Q0, f0);
	}

	void StaticLink::FixW01()
	{
		W0 = V0 - VA;
		W1 = V1 - VB;
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
			auto s2 = window[2].state;
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
	bool FindAsRoot(ScriptedLink& link, FReal t0, FReal t1, FReal v0, FReal v1, FReal DTOL, FReal TTOL)
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

	bool FindMinimum(ScriptedLink& link, FReal t0, FReal t1, FReal v0, FReal v1, FReal DTOL, FReal TTOL)
	{
		struct Params
		{
			ScriptedLink& link; 
			FReal t0;
			FReal t1;
		};
		auto params = Params{ link, t0, t1 };

		// mirror the problem to positive space
		v0 = Math::Abs(v0);
		v1 = Math::Abs(v1);

		// function to minimize
		auto F = gsl_multimin_function();
		F.params = &params;
		F.n = 1;
		F.f = [](const gsl_vector* v, void* params)->double
		{
			auto p = (Params*)params;

			auto t = gsl_vector_get(v, 0);
			if (!(t >= p->t0 && t <= p->t1))
			{
				return NAN;
			}

			if (!p->link.Find_t(t))
			{ 
				return NAN; 
			}
			return Math::Abs(t - p->link.t1);
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
		gsl_set_error_handler_off();
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
	void FindLinks(std::vector<Link>& links, const StaticLinkConfig& cfg, const std::vector<FReal>& f0s)
	{
		for (auto f0 : f0s)
		{
			auto link = Utiles::StaticLink(cfg, f0);
			if (link.Find_t())
			{
				link.FixParams();
				links.push_back(std::move(link));
			}
		}
	}

	void FindLinks(std::vector<Link>& links, const ScriptedLinkConfig& cfg, FReal f0)
	{
		auto link = Utiles::ScriptedLink(cfg, f0);

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

	void FindLinks(std::vector<Link>& links, const ScriptedLinkConfig& cfg, const std::vector<FReal>& f0s)
	{
		for (auto f0 : f0s)
		{
			FindLinks(links, cfg, f0);
		}
	}
}

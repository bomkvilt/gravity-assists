#include "link.hpp"
#include "keplerOrbit.hpp"
#include <array>

static auto dbgdt = std::vector<float>();

namespace Pathfinder::Link::Utiles
{
	namespace kepel = ::Pathfinder::Kepler::Elliptic;

	LinkAdapter::LinkAdapter(const FindLinksConfig& cfg, float f0)
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

	bool LinkAdapter::Find_t(float t)
	{
		B.SetTime(t);
		R1 = B.GetLocation();
		r1 = R1.Size();
		Q1 = Q0 + Math::Angle2(R0, R1);

		auto [res, bOK] = kepel::epwqq(r0, r1, Q0, Q1, f0);
		if (bOK)
		{
			e = res.e;
			p = res.p;
			w = res.w;
			q0 = res.q0;
			q1 = res.q1;

			a  = kepel::a(e, p);
			E0 = kepel::E(q0, r0, e, a);
			E1 = kepel::E(q1, r1, e, a);
			M0 = kepel::M(E0, e);
			M1 = kepel::M(E1, e);
			dt = kepel::dt(M0, M1, a, GM, bf);
			if (isinf(dt) || isnan(dt))
			{
				return false;
			}
			
			dbgdt.push_back(dt);
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
		auto X = (R0).GetNormal();
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
		W0 = V0 - W0;
		W1 = V1 - W1;
	}
}


namespace Pathfinder::Link::Utiles
{
	struct RootWindowHelper
	{
		enum EState
		{
			  ePositive = 1 << 0
			, eNegative = 1 << 1
			, eZero		= 1 << 2
			, eNAN		= 1 << 3
		};

		struct Point
		{
			EState state = EState::eNAN;
			float  delta = 0;
			float  time = 0;
		};

	public:
		std::array<Point, 2> window;
		float DTOL;

		RootWindowHelper(float DTOL = Math::Epsilon)
			: DTOL(DTOL)
		{}

		void Push(float time, float delta)
		{
			window[1] = window[0];
			window[0].state = DeduceState(delta);
			window[0].delta = delta;
			window[0].time = time;
		}

		EState DeduceState(float delta)
		{
			if (delta == NAN)
			{
				return EState::eNAN;
			}
			if (Math::Equal(delta, 0, DTOL))
			{
				return EState::eZero;
			}
			return delta > 0
				? EState::ePositive
				: EState::eNegative
				;
		}

		auto GetRoot()->std::tuple<Point, Point, bool>
		{
			auto s0 = window[0].state;
			auto s1 = window[1].state;
			if (s0 == EState::eZero)
			{
				auto p = window[0];
				return { p, p, true };
			}
			if ((s0 | s1) == (EState::eNegative | EState::ePositive))
			{
				auto p0 = window[0];
				auto p1 = window[1];
				return { p1, p0, true };
			}
			return { {}, {}, false };
		}

		bool CheckRoot()
		{
			auto s0 = window[0].state;
			auto s1 = window[1].state;
			if (s0 == EState::eZero)
			{
				return true;
			}
			if ((s0 | s1) == (EState::eNegative | EState::ePositive))
			{
				return true;
			}
			return false;
		}
	};
}


namespace Pathfinder::Link::Utiles
{
	bool FindRoot(LinkAdapter& link, float t0, float t1, float v0, float v1, float DTOL, float TTOL)
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
		} while (Abs(t1 - t0) < TTOL);

		return false;
	}
}


namespace Pathfinder::Link
{
	auto FindLinks(std::vector<Link>& links, const FindLinksConfig& cfg, float f0)
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

			auto [p0, p1, _] = window.GetRoot();
			if (!Utiles::FindRoot(link, p0.time, p1.time, p0.delta, p1.delta, cfg.tt, cfg.tt))
			{
				continue;
			}
			link.W0 = cfg.A.GetVelocity();
			link.W1 = cfg.B.GetVelocity();
			links.push_back(link);
		}
	}

	void FindLinks(std::vector<Link>& links, const FindLinksConfig& cfg, const std::vector<float>& f0s)
	{
		for (auto f0 : f0s)
		{
			FindLinks(links, cfg, f0);
		}
	}
}

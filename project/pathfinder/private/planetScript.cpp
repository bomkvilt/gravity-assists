#include "planetScript.hpp"
#include "SpiceUsr.h"
#include <array>


namespace Pathfinder::PlanetScript::utiles
{
	std::string GetSPICEName(EPlanet planetName)
    {
        switch (planetName) {
		case EPlanet::eNONE		: return "";
		case EPlanet::eSun		: return "Sun";
		case EPlanet::eMercury	: return "Mercury";
		case EPlanet::eVenus	: return "Venus";
		case EPlanet::eEarth	: return "Earth";
		case EPlanet::eMoon		: return "Moon";
		case EPlanet::eMars		: return "Mars";
		case EPlanet::eJupter	: return "Jupiter";
		default: 
			throw std::out_of_range("Unsupported planet");
		};
    }

	EPlanet GetCenterBody(EPlanet planetName)
	{
		switch (planetName) {
		case EPlanet::eSun		: return EPlanet::eNONE;
		case EPlanet::eMercury	: return EPlanet::eSun;
		case EPlanet::eVenus	: return EPlanet::eSun;
		case EPlanet::eEarth	: return EPlanet::eSun;
		case EPlanet::eMoon		: return EPlanet::eEarth;
		case EPlanet::eMars		: return EPlanet::eSun;
		case EPlanet::eJupter	: return EPlanet::eSun;
		default:
			throw std::out_of_range("Unsupported planet");
		}
	}

	class SPICE : boost::noncopyable
	{
	public:
		static SPICE& Get()
		{
			static SPICE spice;
			return spice;
		}

		FVector GetLocation(const std::string& name, float time)
		{
			auto state = GetRawMovement(name, time);
			// km -> m
			return { 
				(float)state[0] * 1e3f, 
				(float)state[1] * 1e3f, 
				(float)state[2] * 1e3f
			};
		}

		FVector GetVelocity(const std::string& name, float time)
		{
			auto state = GetRawMovement(name, time);
			// km/s -> m/s
			return {
				(float)state[3] * 1e3f,
				(float)state[4] * 1e3f,
				(float)state[5] * 1e3f
			};
		}

		auto GetMovement(const std::string& name, float time)->std::tuple<FVector, FVector>
		{
			auto state = GetRawMovement(name, time);
			// km -> m, km/s -> m/s
			return {{
				(float)state[0] * 1e3f,
				(float)state[1] * 1e3f,
				(float)state[2] * 1e3f
			}, {
				(float)state[3] * 1e3f,
				(float)state[4] * 1e3f,
				(float)state[5] * 1e3f
			}};
		}

		float GetGM(const std::string& name)
		{
			// km^3/s^2 -> m^3/s^2
			return GetRawGM(name) * 1e9f;
		}

		float GetAbsTime(const std::string& time)
		{
			SpiceDouble et = 0;
			str2et_c(time.c_str(), &et);
			assert(et);
			return float(et);
		}

		float GetPeriod(const std::string& body, const std::string& primatyBody, float time)
		{ 
			// \see: https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/C/cspice/oscltx_c.html
			auto state = GetRawMovement(body, time);
			auto gm    = GetRawGM(primatyBody);
			SpiceDouble params[20];
			oscltx_c(&state.front(), time, gm, params);
			return params[10];
		}

	private:
		SPICE()
		{
			// generic planet ephemerids
			furnsh_c(SPICE_KERNELS"/de438.bsp");
			furnsh_c(SPICE_KERNELS"/jup343.bsp");
			furnsh_c(SPICE_KERNELS"/mar097.bsp");
			furnsh_c(SPICE_KERNELS"/latest_leapseconds.tls");
			furnsh_c(SPICE_KERNELS"/gm_de431.tpc");
		}

		// position + velocity in km
		std::array<SpiceDouble,6> GetRawMovement(const std::string& name, float time)
		{
			auto state = std::array<SpiceDouble,6>();
			SpiceDouble lightTime = 0;
			spkezr_c(name.c_str(), time, "J2000", "NONE", "SSB", &state.front(), &lightTime);
			return state;
		}

		SpiceDouble GetRawGM(const std::string& name)
		{
			SpiceInt n = 0;
			SpiceDouble gm = 0;
			bodvrd_c(name.c_str(), "GM", 1, &n, &gm);
			assert(n > 0);
			return gm;
		}
	};
}


namespace Pathfinder::PlanetScript
{
	PlanetScript::PlanetScript(EPlanet planet, const std::string& J2000Time)
		: name  (utiles::GetSPICEName(planet))
		, center(utiles::GetSPICEName(utiles::GetCenterBody(planet)))
		, t0	(utiles::SPICE::Get().GetAbsTime(J2000Time))
		, GM	(utiles::SPICE::Get().GetGM(name))
	{
		if (center != "")
		{
			T = utiles::SPICE::Get().GetPeriod(name, center, t0);
		}
	}

	float PlanetScript::GetT(float time) const
	{
		return T;
	}
	
	float PlanetScript::GetGM(float time) const
	{
		return GM;
	}
	
	FVector PlanetScript::GetLocation(float time) const
	{
		return utiles::SPICE::Get().GetLocation(name, t0 + time);
	}
	
	FVector PlanetScript::GetVelocity(float time) const
	{
		return utiles::SPICE::Get().GetVelocity(name, t0 + time);
	}
	
	auto PlanetScript::GetMovement(float time) -> std::tuple<FVector, FVector> const
	{
		return utiles::SPICE::Get().GetMovement(name, t0 + time);
	}
}

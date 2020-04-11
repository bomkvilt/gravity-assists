#include "planetEphemerides.hpp"
#include "SpiceUsr.h"
#include <array>


namespace PlanetEphemeridesUtiles
{
    std::string EPlanet2SPICE(EPlanet planetName)
    {
        switch (planetName) {
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

	EPlanet GetPrimaryBody(EPlanet planetName)
	{
		switch (planetName) {
		case EPlanet::eSun		: return EPlanet::eNotAPlant;
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
				(float)state[1] * 1e3f, 
				(float)state[2] * 1e3f, 
				(float)state[3] * 1e3f
			};
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


PlanetEphemerides::PlanetEphemerides(EPlanet planetName, const std::string& absTime)
	: name    (PlanetEphemeridesUtiles::EPlanet2SPICE(planetName))
	, timeRoot(PlanetEphemeridesUtiles::SPICE::Get().GetAbsTime(absTime))
{
	auto parent = PlanetEphemeridesUtiles::GetPrimaryBody(planetName);
	primaryBody = PlanetEphemeridesUtiles::EPlanet2SPICE(parent);
	T  = PlanetEphemeridesUtiles::SPICE::Get().GetPeriod(name, primaryBody, timeRoot);
	GM = PlanetEphemeridesUtiles::SPICE::Get().GetGM(name);
}

FVector PlanetEphemerides::GetLocation(float time)
{
	return PlanetEphemeridesUtiles::SPICE::Get().GetLocation(name, time);
}

float PlanetEphemerides::GetGM() const 
{
	return GM;
}

float PlanetEphemerides::GetT() const
{
	return T;
}

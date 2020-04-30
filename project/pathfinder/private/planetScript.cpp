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

		FVector GetLocation(const std::string& name, FReal time)
		{
			auto state = GetRawMovement(name, time);
			// km -> m
			return { 
				(FReal)state[0] * 1e3f, 
				(FReal)state[1] * 1e3f, 
				(FReal)state[2] * 1e3f
			};
		}

		FVector GetVelocity(const std::string& name, FReal time)
		{
			auto state = GetRawMovement(name, time);
			// km/s -> m/s
			return {
				(FReal)state[3] * 1e3f,
				(FReal)state[4] * 1e3f,
				(FReal)state[5] * 1e3f
			};
		}

		auto GetMovement(const std::string& name, FReal time)->std::tuple<FVector, FVector>
		{
			auto state = GetRawMovement(name, time);
			// km -> m, km/s -> m/s
			return {{
				(FReal)state[0] * 1e3f,
				(FReal)state[1] * 1e3f,
				(FReal)state[2] * 1e3f
			}, {
				(FReal)state[3] * 1e3f,
				(FReal)state[4] * 1e3f,
				(FReal)state[5] * 1e3f
			}};
		}

		FReal GetGM(const std::string& name)
		{
			// km^3/s^2 -> m^3/s^2
			return GetRawGM(name) * 1e9f;
		}

		FReal GetAbsTime(const std::string& time)
		{
			SpiceDouble et = 0;
			str2et_c(time.c_str(), &et);
			assert(et);
			return FReal(et);
		}

		FReal GetPeriod(const std::string& body, const std::string& primatyBody, FReal time)
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
		std::array<SpiceDouble,6> GetRawMovement(const std::string& name, FReal time)
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

	FReal PlanetScript::GetT(FReal time) const
	{
		return T;
	}
	
	FReal PlanetScript::GetGM(FReal time) const
	{
		return GM;
	}
	
	FVector PlanetScript::GetLocation(FReal time) const
	{
		if (IsDiscret())
		{
			auto& [r, v] = GetMovement_D(time);
			return r;
		}
		return GetLocation_C(time);
	}
	
	FVector PlanetScript::GetVelocity(FReal time) const
	{
		if (IsDiscret())
		{
			auto& [r, v] = GetMovement_D(time);
			return v;
		}
		return GetVelocity_C(time);
	}
	
	auto PlanetScript::GetMovement(FReal time) -> std::tuple<FVector, FVector> const
	{
		if (IsDiscret())
		{
			return GetMovement_D(time);
		}
		return GetMovement_C(time);
	}

	void PlanetScript::MakeDiscret(FReal stepSize_, FReal chunkSize_)
	{
		if (IsDiscret())
		{
			throw std::runtime_error("the ephemerides are already discret");
		}
		
		if (stepSize_ > 0 && chunkSize_ > 0)
		{
			stepSize = stepSize_;
			chunkSize = chunkSize_;
		}
		else
		{
			throw std::runtime_error("steps and chunk sizes must be positive");
		}
	}

	bool PlanetScript::IsDiscret() const
	{
		return stepSize > 0;
	}

	const PlanetScript::MovState& PlanetScript::GetMovement_D(FReal time) const
	{
		UInt64 chunkN = time / chunkSize;
		UInt64 blockN = time / stepSize;

		auto pos = chunks.find(chunkN);
		auto end = chunks.end();
		if (pos != end)
		{
			return pos->second.at(blockN);
		}
		
		const_cast<PlanetScript*>(this)->AddChunk(chunkN);

		pos = chunks.find(chunkN);
		end = chunks.end();
		if (pos != end)
		{
			return pos->second.at(blockN);
		}		
		throw std::runtime_error("cannot get discret value fot t=" + std::to_string(time) + " s");
	}

	FVector PlanetScript::GetLocation_C(FReal time) const
	{
		return utiles::SPICE::Get().GetLocation(name, t0 + time);
	}

	FVector PlanetScript::GetVelocity_C(FReal time) const
	{
		return utiles::SPICE::Get().GetVelocity(name, t0 + time);
	}

	auto PlanetScript::GetMovement_C(FReal time) -> std::tuple<FVector, FVector> const
	{
		return utiles::SPICE::Get().GetMovement(name, t0 + time);
	}

	void PlanetScript::AddChunk(UInt64 chunkN)
	{
		auto& chunk = chunks[chunkN];
		FReal ti = chunkSize * chunkN;
		FReal t1 = chunkSize + ti;
		UInt64 N = ti / stepSize;
		for (; ti < t1; ti += stepSize, ++N)
		{
			chunk[N] = GetMovement_C(ti);
		}
	}
}

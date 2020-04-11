#ifndef PATHFINDER__FIRSTAPPROXUTILES_HPP
#define PATHFINDER__FIRSTAPPROXUTILES_HPP

#include "math/math.hpp"
#include "planetBase.hpp"


// functions to find orbital params
namespace FirstApproxUtiles::finders
{
	auto FindPEW(float Q0, float Q1, float r0, float r1, float f0)->std::tuple<float, float, float, bool>;
	auto FindPE (float Q0, float Q1, float r0, float r1, float w )->std::tuple<float, float, bool>;

	float FindPeriapsisAngle(float Q0, float Q1, float r0, float r1, float f0);
	float FindParameter		(float Q0, float Q1, float r0, float r1, float w );
	float FindEccentricity	(float Q0, float Q1, float r0, float r1, float w );
	float FindTime			(float Q0, float Q1, float M , float p);
	float FindVelocity(float q, float w, float e, float p, float M);
	float FindVelAngle(float q, float w, float e);
}


namespace FirstApproxUtiles
{
	class TragectoryFinder
	{
	public:
		FVector R0;   FVector R1;
		float r0 = 0; float r1 = 0;
		float f0 = 0; float f1 = 0;
		float v0 = 0; float v1 = 0;
		float t0 = 0; float t1 = 0; float te = 0; float ts = 0;
		float Q0 = 0; float Q1 = 0;
		float e = 0;
		float p = 0;
		float w = 0;
		float M = 0;

	public:
		TragectoryFinder(const FVector& R0, float Q0, float t0, float te, float ts, float M);
		bool FindTragectory(PlanetBase& B, float f0_);
	};
}


#endif //!PATHFINDER__FIRSTAPPROXUTILES_HPP

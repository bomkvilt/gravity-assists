#ifndef PATHFINDER__LINKS_HPP
#define PATHFINDER__LINKS_HPP

#include "math/math.hpp"


namespace Pathfinder::Link
{
	struct Link
	{
		FVector R0;		FVector R1;
		FVector V0;		FVector V1;
		FVector W0;		FVector W1;
		float r0 = 0;	float r1 = 0;
		float f0 = 0;	float f1 = 0;
		float v0 = 0;	float v1 = 0;
		float t0 = 0;	float t1 = 0;
		float Q0 = 0;	float Q1 = 0;
		float q0 = 0;	float q1 = 0;
		float E0 = 0;	float E1 = 0;
		float M0 = 0;	float M1 = 0;

		float dt = 0;
		float e = 0;
		float p = 0;
		float a = 0;
		float w = 0;
		bool bf = false;
	};
}


#endif //!PATHFINDER__LINKS_HPP

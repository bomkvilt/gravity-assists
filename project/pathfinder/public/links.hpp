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
		FReal r0 = 0;	FReal r1 = 0;
		FReal f0 = 0;	FReal f1 = 0;
		FReal v0 = 0;	FReal v1 = 0;
		FReal t0 = 0;	FReal t1 = 0;
		FReal Q0 = 0;	FReal Q1 = 0;
		FReal q0 = 0;	FReal q1 = 0;
		FReal E0 = 0;	FReal E1 = 0;
		FReal M0 = 0;	FReal M1 = 0;

		FReal dt = 0;
		FReal e = 0;
		FReal p = 0;
		FReal a = 0;
		FReal w = 0;
		bool bf = false;

		FVector GetAxisX() const;
		FVector GetAxisY() const;
		FVector GetAxisZ() const;
		FVector GetTragectoryPoint(FReal q) const;
		FReal   GetRealAnomaly(FReal fraction) const;
		FReal   GetTossAngle(FReal q) const;
	};
}


#endif //!PATHFINDER__LINKS_HPP

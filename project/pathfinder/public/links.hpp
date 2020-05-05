#ifndef PATHFINDER__LINKS_HPP
#define PATHFINDER__LINKS_HPP

#include "math/math.hpp"


namespace Pathfinder::Link
{
	struct Link : public reflect::FArchived
	{
		ARCH_BEGIN(reflect::FArchived)
			ARCH_FIELD(, , R0) ARCH_FIELD(, , R1)
			ARCH_FIELD(, , V0) ARCH_FIELD(, , V1)
			ARCH_FIELD(, , W0) ARCH_FIELD(, , W1)
			ARCH_FIELD(, , r0) ARCH_FIELD(, , r1)
			ARCH_FIELD(, , f0) ARCH_FIELD(, , f1)
			ARCH_FIELD(, , v0) ARCH_FIELD(, , v1)
			ARCH_FIELD(, , t0) ARCH_FIELD(, , t1)
			ARCH_FIELD(, , Q0) ARCH_FIELD(, , Q1)
			ARCH_FIELD(, , q0) ARCH_FIELD(, , q1)
			ARCH_FIELD(, , E0) ARCH_FIELD(, , E1)
			ARCH_FIELD(, , M0) ARCH_FIELD(, , M1)
			ARCH_FIELD(, , dt)
			ARCH_FIELD(, , e )
			ARCH_FIELD(, , p )
			ARCH_FIELD(, , a )
			ARCH_FIELD(, , w )
			ARCH_FIELD(, , bf)
			ARCH_END()
	public:
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

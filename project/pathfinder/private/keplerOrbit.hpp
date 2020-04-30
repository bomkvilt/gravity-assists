#ifndef PATHFINDER__KEPLERORBIT_HPP
#define PATHFINDER__KEPLERORBIT_HPP

#include <tuple>
#include "math/math.hpp"


namespace Pathfinder::Kepler
{
	FReal h(FReal vi, FReal ri, FReal GM);
	FReal v(FReal hi, FReal ri, FReal GM);

	FReal reorbit_v(FReal v0, FReal r0, FReal r1, FReal GM);
}

namespace Pathfinder::Kepler::Elliptic
{
	struct _epwqq
	{
		FReal e  = 0;
		FReal p  = 0;
		FReal w  = 0;
		FReal q0 = 0;
		FReal q1 = 0;
	};
	auto epwqq(FReal r0, FReal r1, FReal Q0, FReal Q1, FReal f0)->std::tuple<_epwqq, bool>;
	auto ep(FReal r0, FReal r1, FReal q0, FReal q1)->std::tuple<FReal, FReal>;
	auto qq(FReal Q0, FReal Q1, FReal w           )->std::tuple<FReal, FReal>;
	FReal w(FReal r0, FReal r1, FReal Q0, FReal Q1, FReal f0);

	bool  bf(FReal Q0, FReal f0);
	FReal a (FReal e , FReal p );
	FReal E (FReal qi, FReal ri, FReal e);
	FReal M (FReal Ei, FReal e);
	FReal dt(FReal M0, FReal M1, FReal a, FReal GM, bool bf);
	FReal v (FReal qi, FReal e , FReal p, FReal GM);
	FReal f (FReal Qi, FReal qi, FReal e, bool bf);

	FReal NZ(FReal f);
}

namespace Pathfinder::Kepler::Hiperbolic
{
	FReal bmin(FReal v, FReal r, FReal rpl, FReal GM);

	FReal kink(FReal v, FReal b, FReal r, FReal GM);
}


#endif //!PATHFINDER__KEPLERORBIT_HPP

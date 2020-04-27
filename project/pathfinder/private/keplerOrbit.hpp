#ifndef PATHFINDER__KEPLERORBIT_HPP
#define PATHFINDER__KEPLERORBIT_HPP

#include <tuple>


namespace Pathfinder::Kepler
{
	float h(float vi, float ri, float GM);

	float reorbit_v(float v0, float r0, float r1, float GM);
}

namespace Pathfinder::Kepler::Elliptic
{
	struct _epwqq
	{
		float e  = 0;
		float p  = 0;
		float w  = 0;
		float q0 = 0;
		float q1 = 0;
	};
	auto epwqq(float r0, float r1, float Q0, float Q1, float f0)->std::tuple<_epwqq, bool>;
	auto ep(float r0, float r1, float q0, float q1)->std::tuple<float, float>;
	auto qq(float Q0, float Q1, float w           )->std::tuple<float, float>;
	float w(float r0, float r1, float Q0, float Q1, float f0);

	bool  bf(float Q0, float f0);
	float a (float e , float p );
	float E (float qi, float ri, float e, float a);
	float M (float Ei, float e);
	float dt(float M0, float M1, float a, float GM, bool bf);
	float v (float qi, float e , float p, float GM);
	float f (float Qi, float qi, float e, bool bf);

	float NZ(float f);
}

namespace Pathfinder::Kepler::Hiperbolic
{
	float bmin(float v, float r, float rpl, float GM);

	float kink(float v, float b, float r, float GM);
}


#endif //!PATHFINDER__KEPLERORBIT_HPP

#ifndef PATHFINDER__LINK_HPP
#define PATHFINDER__LINK_HPP

#include "math/math.hpp"
#include "ephemeridesClient.hpp"
#include "links.hpp"


namespace Pathfinder::Link
{
	struct FindLinksConfig
	{
		Ephemerides::EphemeridesClient A;
		Ephemerides::EphemeridesClient B;
		float t0 = 0;
		float te = 0;
		float ts = 0;
		float tt = 0;
		float GM = 0;
	};
	
	void FindLinks(std::vector<Link>& links, const FindLinksConfig& cfg, const std::vector<float>& f0s);
}


namespace Pathfinder::Link::Utiles
{
	struct LinkAdapter : public Link
	{
		Ephemerides::EphemeridesClient A;
		Ephemerides::EphemeridesClient B;
		float GM;

	public:

		LinkAdapter(const FindLinksConfig& cfg, float f0);

		bool Find_t(float t);

		void FixParams();
		void Fix2DParams();
		void Fix3DParams();
	};
}


#endif //!PATHFINDER__LINK_HPP

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
		FReal t0 = 0; // [s] - scan range begin
		FReal te = 0; // [s] - scan range end
		FReal ts = 0; // [s] - mismatch function sign change scan step
		FReal tt = 0; // [s] - mismatch tolerance
		FReal td = 0; // [s] - 
		FReal GM = 0; // [m3/s2]
	};
	
	void FindLinks(std::vector<Link>& links, const FindLinksConfig& cfg, const std::vector<FReal>& f0s);
}


namespace Pathfinder::Link::Utiles
{
	struct LinkAdapter : public Link
	{
		Ephemerides::EphemeridesClient A;
		Ephemerides::EphemeridesClient B;
		FReal GM;

	public:

		LinkAdapter(const FindLinksConfig& cfg, FReal f0);

		bool Find_t(FReal t);

		void FixParams();
		void Fix2DParams();
		void Fix3DParams();
	};
}


#endif //!PATHFINDER__LINK_HPP

#ifndef PATHFINDER__LINK_HPP
#define PATHFINDER__LINK_HPP

#include "links.hpp"
#include "trajectory/ephemeridesClient.hpp"



namespace Pathfinder::Link
{
	struct ScriptedLinkConfig
	{
		FVector RA;
		FVector VA;
		Ephemerides::EphemeridesClient B;
		FReal t0 = NAN; // [s] - scan range begin
		FReal te = NAN; // [s] - scan range end
		FReal ts = NAN; // [s] - mismatch function sign change scan step
		FReal tt = NAN; // [s] - mismatch tolerance
		FReal td = NAN; // [s] - 
		FReal GM = NAN; // [m3/s2]

		void SetA(Ephemerides::IEphemerides& script);
		void SetB(Ephemerides::IEphemerides& script);
		void SetA(Ephemerides::IEphemerides::ptr& script);
		void SetB(Ephemerides::IEphemerides::ptr& script);
	};

	struct StaticLinkConfig
	{
		FVector RA; FVector VA;
		FVector RB; FVector VB;
		FReal t0 = NAN;
		FReal GM = NAN;

		void SetA(Ephemerides::IEphemerides& script);
		void SetA(Ephemerides::IEphemerides::ptr& script);
	};
}


namespace Pathfinder::Link::Utiles
{
	struct LinkAdapter : public Link
	{
		FReal GM;
		
		LinkAdapter(FReal GM);

		bool FixParams();
		void Fix2DParams();
		void Fix3DParams();
		virtual void FixW01() = 0;

		bool Find_t();
	};

	struct ScriptedLink : public LinkAdapter
	{
		FVector VA;
		Ephemerides::EphemeridesClient B;

		ScriptedLink(const ScriptedLinkConfig& conf, FReal f0);

		void FixW01() override;

		bool Find_t(FReal t);
	};

	struct StaticLink : public LinkAdapter
	{
		FVector VA;
		FVector VB;

		StaticLink(const StaticLinkConfig& conf, FReal f0);

		void FixW01() override;
	};
}


namespace Pathfinder::Link
{
	void FindLinks(std::vector<Link>& links, const StaticLinkConfig  & cfg, const std::vector<FReal>& f0s);
	void FindLinks(std::vector<Link>& links, const ScriptedLinkConfig& cfg, const std::vector<FReal>& f0s);
}


#endif //!PATHFINDER__LINK_HPP

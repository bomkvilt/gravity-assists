#ifndef PATHFINDER__EPHEMERIDESCLIENT_HPP
#define PATHFINDER__EPHEMERIDESCLIENT_HPP

#include "interfaces/ephemerides.hpp"



namespace Pathfinder::Ephemerides
{
	// EpehemeridesClient is a default epehemrides db client
	class EphemeridesClient final
	{
	public:
		EphemeridesClient(IEphemerides* conn = nullptr);
		EphemeridesClient(IEphemerides::ptr& conn);

		void SetTime(FReal newTime);
		void SetDriver(IEphemerides* newConn);

		FReal	GetT()  const;
		FReal	GetGM() const;
		FVector GetLocation() const;
		FVector GetVelocity() const;
		auto	GetMovement()->std::tuple<FVector, FVector> const;
		
		bool IsValid() const;

	private:
		IEphemerides* conn;
		FReal time;
	};
}


#endif //!PATHFINDER__EPHEMERIDESCLIENT_HPP

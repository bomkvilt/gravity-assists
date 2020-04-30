#include "ephemeridesClient.hpp"



namespace Pathfinder::Ephemerides
{
	EphemeridesClient::EphemeridesClient(IEphemerides* conn)
		: conn(conn)
		, time(NAN)
	{}

	EphemeridesClient::EphemeridesClient(IEphemerides::ptr& conn)
		: conn(&*conn)
		, time(NAN)
	{}

	void EphemeridesClient::SetTime(FReal newTime)
	{
		time = newTime;
	}

	void EphemeridesClient::SetDriver(IEphemerides* newConn)
	{
		conn = newConn;
	}
	
	FReal EphemeridesClient::GetT() const
	{
		if (IsValid())
		{
			return conn->GetT(time);
		}
		return 0;
	}
	
	FReal EphemeridesClient::GetGM() const
	{
		if (IsValid())
		{
			return conn->GetGM(time);
		}
		return 0;
	}
	
	FVector EphemeridesClient::GetLocation() const
	{
		if (IsValid())
		{
			return conn->GetLocation(time);
		}
		return {};
	}
	
	FVector EphemeridesClient::GetVelocity() const
	{
		if (IsValid())
		{
			return conn->GetVelocity(time);
		}
		return {};
	}
	
	auto EphemeridesClient::GetMovement()->std::tuple<FVector, FVector> const
	{
		if (IsValid())
		{
			return conn->GetMovement(time);
		}
		return {};
	}

	bool EphemeridesClient::IsValid() const
	{
		return conn && !isnan(time);
	}
}

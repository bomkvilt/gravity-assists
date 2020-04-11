#ifndef PATHFINDER__BODY_HPP
#define PATHFINDER__BODY_HPP

#include "math/math.hpp"


class Body
{
public:
	void SetGravParam(float newGrav) { grav = newGrav; }
	float GetGravParam() const { return grav; }

	FVector& GetLocation() { return location; }
	FVector& GetVelocity() { return velocity; }
	const FVector& GetLocation() const { return location; }
	const FVector& GetVelocity() const { return velocity; }

	void SetLocation(const FVector& newLocation) { location = newLocation; }
	void SetVelocity(const FVector& newVelocity) { velocity = newVelocity; }

protected:

	float grav = 0;
	FVector location;
	FVector velocity;
};


#endif //!PATHFINDER__BODY_HPP

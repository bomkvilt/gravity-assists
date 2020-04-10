#ifndef PATHFINDER__BODY_HPP
#define PATHFINDER__BODY_HPP

#include "math/math.hpp"


class Body
{
public:
	Body(float mass = 0) : mass(mass) {}

	virtual ~Body() = default;

public:
	float GetMass()      const { return mass; }
	float GetGravParam() const { return grav; }

	FVector& GetLocation() { return location; }
	FVector& GetVelocity() { return velocity; }
	const FVector& GetLocation() const { return location; }
	const FVector& GetVelocity() const { return velocity; }

	void SetLocation(const FVector& newLocation) { location = newLocation; }
	void SetVelocity(const FVector& newVelocity) { velocity = newVelocity; }

protected:
	float mass = 0;
	float grav = 0;
	FVector location;
	FVector velocity;
};


#endif //!PATHFINDER__BODY_HPP

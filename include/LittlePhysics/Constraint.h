#pragma once
#include "Core.h"
#include "Body.h"
#include "Contact.h"

namespace LP {

	// lower case for math

	struct LP_API normalConstraint
	{
		// assigned values
		Vec2 c1;
		float a1;
		Vec2 c2;
		float a2;

		// calculated by step
		float bias;
		float impulse;
		float mc;
	};

	struct LP_API frictionConstraint
	{
		// assigned values
		Vec2 c1;
		float a1;
		Vec2 c2;
		float a2;

		// calculated by step
		float impulse;
		float mc;
	};

	struct LP_API Contact;

	struct LP_API ContactEdge
	{
		Body* Other;
		Contact* Contact;
		ContactEdge* Prev;
		ContactEdge* Next;
	};

	struct LP_API ContactVelocityConstraint
	{
		Vec2 r1;
		Vec2 r2;
		float depth;
		normalConstraint nc;
		frictionConstraint fc;
	};

	struct LP_API ContactPositionConstraint
	{
		Vec2 r1;
		Vec2 r2;
		float depth;
		Vec2 normal;
	};

	struct LP_API Contact
	{
		Body* body1;
		Body* body2;
		uint32 index1;
		uint32 index2;
		Vec2 normal;
		ContactIdentifier cID;

		// Link lists
		Contact* m_Prev = nullptr;
		Contact* m_Next = nullptr;

		// For Body
		ContactEdge ContactEdge1;
		ContactEdge ContactEdge2;

		// Inverse
		float m1;
		float i1;
		float m2;
		float i2;

		// Constraints
		uint32 count = 1;
		ContactVelocityConstraint vc[2];
		ContactPositionConstraint pc[2];
		// For block solver
		Mat2x2 mc;
		Mat2x2 mcInv;
		// For position constraints
		Vec2 Points[2];
		CONTACT_TYPE type;
		Vec2 localPoints[2];
		// For circles, this is the radius
		// For edges, this is the relative radius
	};

}
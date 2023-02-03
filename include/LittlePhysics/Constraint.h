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

	struct LP_API ContactConstraint
	{
		Vec2 r1;
		Vec2 r2;
		float depth;
		normalConstraint nc;
		frictionConstraint fc;
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
		uint8 count = 1;
		ContactConstraint cc[2];
	};



	struct LP_API NormalConstraint
	{
		// Inputs
		Vec2 R1;
		Vec2 R2;
		Vec2 N;
		Body* Body1;
		Body* Body2;
		uint32 index1;
		uint32 index2;
		float Depth;
		float impulse = 0.0f;
		float bias;

		// Outputs
		Vec2 C1;
		float A1;
		Vec2 C2;
		float A2;
		Velocity  B1;
		Velocity  B2;
	};

	struct LP_API FrictionConstraint
	{

		Vec2 R1;
		Vec2 R2;
		Vec2 N;
		Body* Body1;
		Body* Body2;
		uint32 index1;
		uint32 index2;
		float Depth;
		float impulse = 0.0f;

		// Outputs
		Vec2 C1;
		float A1;
		Vec2 C2;
		float A2;
		Velocity  B1;
		Velocity  B2;
	};

}
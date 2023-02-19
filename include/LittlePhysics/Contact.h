#pragma once
#include "Core.h"
#include "Body.h"
#include "CollisionNarrowPhase.h"

namespace LP {

	enum CONTACT_COMBINATION : uint8
	{
		CIRCLE_CIRCLE, CIRCLE_BOX, CIRCLE_POLYGON, BOX_BOX, BOX_POLYGON, POLYGON_POLYGON
	};

	struct LP_API ContactFeature
	{
		uint8 Type;
		uint8 Edge1;
		uint8 Edge2;
		uint8 Order;
	};

	union LP_API ContactIdentifier
	{
		uint32 ID;
		ContactFeature Feature;
	};

	enum class CONTACT_TYPE
	{
		CIRCLES, EDGE_A, EDGE_B
	};
	struct LP_API ContactInfo
	{
		Vec2 Points[2];
		Vec2 Normal;
		float Depths[2];
		CONTACT_TYPE Type;
		ContactIdentifier Key;
		uint32 Count;

		// if type is CIRCLES the x values are the radius 
		// else this is the clip edge radius from center
		Vec2 RefPoints[2]; 
		Vec2 IncPoints[2];
	};

	struct LP_API ContactDebug
	{
		Body* BodyA;
		Body* BodyB;
		ContactInfo info;

	};
}
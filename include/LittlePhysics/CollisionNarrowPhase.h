#pragma once

#include "Core.h"
#include "Shape.h"
#include "Contact.h"

namespace LP {


	// TODO: Multiple contact points
	// TODO: Contact types
	
	
	//* Collision Tests *//
	bool LP_API TestCollision(const Circle* circleA, const Circle* circleB, const Transform& transA, const Transform& transB);
	bool LP_API TestCollision(const Circle* circle, const AABB* aabb, const Transform& transA, const Transform& transB);
	bool LP_API TestCollision(const Circle* circle, const Box* box, const Transform& transA, const Transform& transB);
	//bool LP_API TestCollision(const Circle* circle, const Line* line, const Transform& transA, const Transform& transB);
	bool LP_API TestCollision(const Circle* circle, const Polygon* poly, const Transform& transA, const Transform& transB);
	bool LP_API TestCollision(const AABB* aabb1, const AABB* aabb2, const Transform& transA, const Transform& transB);
	bool LP_API TestCollision(const Box* box1, const Box* box2, const Transform& transA, const Transform& transB);
	//bool LP_API TestCollision(const Box* box, const Line* line, const Transform& transA, const Transform& transB);
	bool LP_API TestCollision(const Box* box, const Polygon* poly, const Transform& transA, const Transform& transB);
	//bool LP_API TestCollision(const Line* line1, const Line* line2, const Transform& transA, const Transform& transB);
	//bool LP_API TestCollision(const Line* line, const Polygon* poly, const Transform& transA, const Transform& transB);
	bool LP_API TestCollision(const Polygon* poly1, const Polygon* poly2, const Transform& transA, const Transform& transB);

	//* Find ContactInfo *//
	bool LP_API FindCollision(ContactInfo* info, const Circle* circleA, const Circle* circleB, const Transform& transA, const Transform& transB);
	//bool LP_API FindCollision(ContactInfo* info, const Circle* circle, const AABB* aabb, const Transform& transA, const Transform& transB);
	bool LP_API FindCollision(ContactInfo* info, const Circle* circle, const Box* box, const Transform& transA, const Transform& transB);
	//bool LP_API FindCollision(ContactInfo* info, const Circle* circle, const Line* line, const Transform& transA, const Transform& transB);
	bool LP_API FindCollision(ContactInfo* info, const Circle* circle, const Polygon* poly, const Transform& transA, const Transform& transB);
	//bool LP_API FindCollision(ContactInfo* info, const AABB* aabb1, const AABB* aabb2, const Transform& transA, const Transform& transB);
	bool LP_API FindCollision(ContactInfo* info, const Box* box1, const Box* box2, const Transform& transA, const Transform& transB);
	//bool LP_API FindCollision(ContactInfo* info, const Box* box, const Line* line, const Transform& transA, const Transform& transB);
	bool LP_API FindCollision(ContactInfo* info, const Box* box, const Polygon* poly, const Transform& transA, const Transform& transB);
	//bool LP_API FindCollision(ContactInfo* info, const Line* line1, const Line* line2, const Transform& transA, const Transform& transB);
	//bool LP_API FindCollision(ContactInfo* info, const Line* line, const Polygon* poly, const Transform& transA, const Transform& transB);
	bool LP_API FindCollision(ContactInfo* info, const Polygon* poly1, const Polygon* poly2, const Transform& transA, const Transform& transB);
}
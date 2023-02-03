#pragma once
#include "Core.h"
#include "Math.h"
#include <vector>

#define LP_POINT_SIZE 8
namespace LP {

	enum class COLLISION_SHAPE_TYPE
	{
		CIRCLE = 0, BOX, POLYGON
	};

	struct LP_API AABB
	{
		Vec2 Min;
		Vec2 Max;
		bool IsIn(const AABB& aabb) const
		{
			if (aabb.Max.x < Max.x) return false;
			if (aabb.Max.y < Max.y) return false;
			if (aabb.Min.x > Min.x) return false;
			if (aabb.Min.y > Min.y) return false;
			return true;
		}
		bool TestOverlap(const AABB& aabb)const
		{
			Vec2 MaxA = Max;
			Vec2 MinA = Min;
			Vec2 MaxB = aabb.Max;
			Vec2 MinB = aabb.Min;
			if (MaxA.y < MinB.y || MaxB.y < MinA.y)
				return false;
			if (MaxA.x < MinB.x || MaxB.x < MinA.x)
				return false;
			return true;
		}
	};

	struct LP_API Shape
	{
		virtual float GetArea() const = 0;
		virtual float GetInertia(float density) const = 0;
		virtual AABB GetAABB(const Transform& tranf) const = 0;
	};

	struct LP_API Circle : public Shape
	{
		virtual float GetArea() const override;
		virtual float GetInertia(float density) const override;
		virtual AABB GetAABB(const Transform& tranf) const override;
		Vec2 Center;
		float Radius;
	};

	struct LP_API Box : public Shape
	{
		virtual float GetArea() const override;
		virtual float GetInertia(float density) const override;
		virtual AABB GetAABB(const Transform& tranf) const override;
		Vec2 Center;
		Vec2 Size;
	};

	struct LP_API Line
	{
		Vec2 Start;
		Vec2 End;
	};

	struct LP_API Polygon : public Shape
	{
		virtual float GetArea() const override;
		virtual float GetInertia(float density) const override;
		virtual AABB GetAABB(const Transform& tranf) const override;
		Vec2 Points[LP_POINT_SIZE];
		uint32 Count;
	};
}
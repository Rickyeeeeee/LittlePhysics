#include <LittlePhysics/Shape.h>

namespace LP {

	float Circle::GetArea() const
	{
		return PI * Radius * Radius;
	}

	float Circle::GetInertia(float density) const
	{
		return 0.5f * GetArea() * density * Radius * Radius;
	}

	AABB Circle::GetAABB(const Transform& tranf) const
	{
		AABB aabb;
		aabb.Max = tranf.P + Vec2{ Radius, Radius };
		aabb.Min = tranf.P - Vec2{ Radius, Radius };
		return aabb;
	}

	float Box::GetArea() const
	{
		return 4 * Size.x * Size.y;
	}

	float Box::GetInertia(float density) const
	{
		return density * Size.x * Size.y * (Size.Dot(Size)) / 3.0f;
	}

	AABB Box::GetAABB(const Transform& tranf) const
	{
		AABB aabb;
		Mat2x2 mat = tranf.R.GetMatrix();
		Vec2 e1 = mat * Size;
		Vec2 e2 = mat * Vec2{ -Size.x, Size.y };
		e1 = { fabs(e1.x), fabs(e1.y) };
		e2 = { fabs(e2.x), fabs(e2.y) };
		if (e1.x > e2.x)
		{
			aabb.Max.x = e1.x;
			aabb.Max.y = e2.y;
			aabb.Min.x = -e1.x;
			aabb.Min.y = -e2.y;
		}
		else
		{
			aabb.Max.x = e2.x;
			aabb.Max.y = e1.y;
			aabb.Min.x = -e2.x;
			aabb.Min.y = -e1.y;
		}
		aabb.Max += tranf.P;
		aabb.Min += tranf.P;
		return aabb;
	}

	float Polygon::GetArea() const
	{
		float area = 0;
		for (uint32 i = 1; i < Count; i++)
		{
			uint32 j = i - 1;
			Vec2 v1 = Points[j];
			Vec2 v2 = Points[i];
			area += fabs(v1.Cross(v2)) / 2.0f;
		}
		return area;
	}

	float Polygon::GetInertia(float density) const
	{
		float intertia = 0.0f;
		for (int i = 1; i < Count; i++)
		{
			int j = i - 1;
			Vec2 v1 = Points[j];
			Vec2 v2 = Points[i];
			float m = density * fabs(v1.Cross(v2)) * 0.5f;
			intertia += m * (v1.Length2() + v2.Length2() + v1.Dot(v2)) / 6.0f;
		}
		return intertia;
	}

	AABB Polygon::GetAABB(const Transform& tranf) const
	{
		AABB aabb;
		aabb.Max = { -HUGE_VALF, -HUGE_VALF };
		aabb.Min = {  HUGE_VALF,  HUGE_VALF };
		Mat2x2 mat = tranf.R.GetMatrix();
		for (uint32 i = 0; i < Count; i++)
		{
			Vec2 p = mat * Points[i];
			aabb.Max.x = fmaxf(aabb.Max.x, p.x);
			aabb.Min.x = fminf(aabb.Min.x, p.x);
			aabb.Max.y = fmaxf(aabb.Max.y, p.y);
			aabb.Min.y = fminf(aabb.Min.y, p.y);
		}
		aabb.Max += tranf.P;
		aabb.Min += tranf.P;
		return aabb;
	}

}

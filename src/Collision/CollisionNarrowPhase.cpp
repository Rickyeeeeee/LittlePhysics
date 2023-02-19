#include <LittlePhysics/CollisionNarrowPhase.h>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <list>
namespace LP 	
{
	bool LP_API TestCollision(const Circle* circleA, const Circle* circleB, const Transform& transA, const Transform& transB)
	{
		Vec2 dist = circleA->Center + transA.P - circleB->Center - transB.P;
		float r = circleA->Radius + circleB->Radius;
		return (dist.x * dist.x + dist.y * dist.y) <= r * r;
	}
	
	bool LP_API TestCollision(const Circle* circle, const AABB* aabb, const Transform& transA, const Transform& transB)
	{
		Vec2 c = circle->Center + transA.P;
		Vec2 max = aabb->Max + transB.P;
		Vec2 min = aabb->Min + transB.P;
		Vec2 p{ 0.0f, 0.0f };
		p.x = std::max(std::min(c.x, max.x), min.x);
		p.y = std::max(std::min(c.y, max.y), min.y);

		return (c - p).Length2() <= (circle->Radius * circle->Radius);
	}

	bool LP_API TestCollision(const Circle* circle, const Box* box, const Transform& transA, const Transform& transB)
	{
		Vec2 c = (-transB.R).GetMatrix() * (circle->Center + transA.P - transB.P - box->Center);
		Vec2 max = box->Size;
		Vec2 min = -box->Size;
		Vec2 p{ 0.0f, 0.0f };
		p.x = std::max(std::min(c.x, max.x), min.x);
		p.y = std::max(std::min(c.y, max.y), min.y);

		return (c - p).Length2() <= (circle->Radius * circle->Radius);
	}

	bool LP_API TestCollision(const AABB* aabb1, const AABB* aabb2, const Transform& transA, const Transform& transB)
	{
		Vec2 MaxA = aabb1->Max + transA.P;
		Vec2 MinA = aabb1->Min + transA.P;
		Vec2 MaxB = aabb2->Max + transB.P;
		Vec2 MinB = aabb2->Min + transB.P;
		if (MaxA.y < MinB.y || MaxB.y < MinA.y)
			return false;
		if (MaxA.x < MinB.x || MaxB.x < MinA.x)
			return false;
		return true;
	}

	// 2D SAT
	// poly vs box can be implemented here;
	bool LP_API TestCollision(const Box* box1, const Box* box2, const Transform& transA, const Transform& transB)
	{
		Vec2 axis[4];
		axis[0] =  transA.R.Axis();
		axis[1] =  transB.R.Axis();
		axis[2] =  transA.R.AxisPer();
		axis[3] =  transB.R.AxisPer();
		//Mat2x2 ra{ transA.R.Cos, -transA.R.Sin, transA.R.Sin, transA.R.Cos };
		Vec2 dist = box2->Center + transB.P - box1->Center - transA.P;
		Mat2x2 ra = transA.R.GetMatrix();
		Vec2 extenda[2];
		extenda[0] = ra * box1->Size;
		extenda[1] = ra * Vec2{ box1->Size.x, -box1->Size.y };
		Mat2x2 rb = transB.R.GetMatrix();
		Vec2 extendb[2];
		extendb[0] = rb * box2->Size;
		extendb[1] = rb * Vec2{ box2->Size.x, -box2->Size.y };
		for (int32 i = 0; i < 4; i++)
		{
			float a = std::max(fabs(axis[i].Dot(extenda[0])), fabs(axis[i].Dot(extenda[1])));
			float b = std::max(fabs(axis[i].Dot(extendb[0])), fabs(axis[i].Dot(extendb[1])));
			float c = fabs(axis[i].Dot(dist));
			if (a + b - c < 0)
				return false;
		}
		return true;
	}

	// GJK
	template <typename SupportFn>
	static inline bool GJK(Vec2 center1, Vec2 center2, SupportFn Support)
	{
		Vec2 d = (center2 - center1).Normalize();
		Vec2 simplex[3];
		uint32 simNum = 0;
		simplex[simNum++] = Support(d);
		d = -simplex[0];
		bool overlap = false;
		while (true)
		{
			Vec2 a = Support(d);
			if (a.Dot(d) < 0.0f)
				break;
			simplex[simNum++] = a;
			if (simNum == 2)
			{
				Vec2 b = simplex[0];
				Vec2 a = simplex[1];
				Vec2 ab = b - a;
				Vec2 ao = -a;
				float cross = ab.x * ao.y - ab.y * ao.x;
				Vec2 abPerp = Vec2(-cross * ab.y, cross * ab.x);
				d = abPerp;
			}
			else
			{
				Vec2 c = simplex[0];
				Vec2 b = simplex[1];
				Vec2 a = simplex[2];
				Vec2 ab = b - a;
				Vec2 ac = c - a;
				Vec2 ao = -a;
				float cross = ac.x * ab.y - ac.y * ab.x;
				Vec2 abPerp = Vec2(-cross * ab.y, cross * ab.x);
				cross = -cross;
				Vec2 acPerp = Vec2(-cross * ac.y, cross * ac.x);
				if (abPerp.Dot(ao) > 0)
				{
					simNum -= 1;
					simplex[0] = simplex[1];
					simplex[1] = simplex[2];
					d = abPerp;
				}
				else if (acPerp.Dot(ao) > 0)
				{
					simNum -= 1;
					simplex[1] = simplex[2];
					d = acPerp;
				}
				else
				{
					overlap = true;
					break;
				}
			}
		}
		return overlap;
	}
	// TODO: fix epa bug

	struct PtNode
	{
		Vec2 point = { 0.0f };
		PtNode* next = nullptr;
	};

	template <typename SupportFn1, typename SupportFn2>
	static inline bool GJKwithEPA(ContactInfo* info, Vec2 center1, Vec2 center2, SupportFn1 SupportA, SupportFn2 SupportB)
	{
		// GJK
		Vec2 d = (center2 - center1).Normalize();
		Vec2 simplex[3];
		uint32 simNum = 0;
		Vec2 p1, p2;
		Vec2 dummy;
		SupportA(d, &p1, &dummy);
		SupportB(-d, &p2, &dummy);
		simplex[simNum++] = p1 - p2;
		d = -simplex[0];
		bool overlap = false;
		while (true)
		{
			SupportA(d, &p1, &dummy);
			SupportB(-d, &p2, &dummy);
			Vec2 at = p1 - p2;
			if (at.Dot(d) < 0.0f)
				break;
			if (d.Length() == 0)
				break;
			simplex[simNum++] = at;
			if (simNum == 2)
			{
				Vec2 b = simplex[0];
				Vec2 a = simplex[1];
				Vec2 ab = b - a;
				Vec2 ao = -a;
				float cross = ab.x * ao.y - ab.y * ao.x;
				Vec2 abPerp = Vec2(-cross * ab.y, cross * ab.x);
				d = abPerp;
			}
			else
			{
				Vec2 c = simplex[0];
				Vec2 b = simplex[1];
				Vec2 a = simplex[2];
				Vec2 ab = b - a;
				Vec2 ac = c - a;
				Vec2 ao = -a;
				float cross = ac.x * ab.y - ac.y * ab.x;
				Vec2 abPerp = Vec2(-cross * ab.y, cross * ab.x);
				cross = -cross;
				Vec2 acPerp = Vec2(-cross * ac.y, cross * ac.x);
				if (abPerp.Dot(ao) > 0.0f)
				{
					simNum -= 1;
					simplex[0] = simplex[1];
					simplex[1] = simplex[2];
					d = abPerp;
				}
				else if (acPerp.Dot(ao) > 0.0f)
				{
					simNum -= 1;
					simplex[1] = simplex[2];
					d = acPerp;
				}
				else
				{
					overlap = true;
					break;
				}
			}
		}
		if (!overlap)
			return false;
		// EPA
		PtNode ptNodePool[7 * 7];
		uint32 ptPoolSize = 0;
		PtNode* polytype = &ptNodePool[ptPoolSize++];
		polytype->point = simplex[0];
		polytype->next = &ptNodePool[ptPoolSize++];
		polytype->next->point = simplex[1];
		polytype->next->next = &ptNodePool[ptPoolSize++];
		polytype->next->next->point = simplex[2];

		PtNode* minIndex = polytype;
		PtNode* minIndexi = polytype;
		float minDistance = HUGE_VALF;
		Vec2 minNormal{ 0.0f };
		int32 iteration = 0;
		while (minDistance == HUGE_VALF && iteration <= 30)
		{
			iteration++;
			if (iteration == 29)
            				return false;
			for (PtNode* i = polytype; i != nullptr; i = i->next)
			{
				PtNode* j;
				if (i->next == nullptr) j = polytype;
				else j = i->next;
				Vec2 vi = i->point;
				Vec2 vj = j->point;
				Vec2 ij = vj - vi;
				float cross = ij.x * vi.y - ij.y * vi.x;
				Vec2 normal = { -cross * ij.y, cross * ij.x };
				normal = normal.Normalize();
				float distance = normal.Dot(vi);
				if (distance < minDistance)
				{
					minDistance = distance;
					minNormal = normal;
					minIndexi = i;
					minIndex = j;
				}

			}
			SupportA(minNormal, &p1, &dummy);
			SupportB(-minNormal, &p2, &dummy);
			Vec2 support = p1 - p2;
			float sd = minNormal.Dot(support);
			if (sd - minDistance > 0.0001f)
			{
				minDistance = HUGE_VALF;
				//polytype.insert(minIndex, support);
				if (minIndex == polytype)
				{
					polytype = &ptNodePool[ptPoolSize++];
					polytype->point = support;
					polytype->next = minIndex;
				}
				else
				{
					minIndexi->next = &ptNodePool[ptPoolSize++];
					minIndexi->next->point = support;
					minIndexi->next->next = minIndex;
				}
			}
		}
		info->Normal = minNormal;
			
		info->Depths[0] = minDistance + 0.00001f;
		return overlap;
	}

	bool LP_API TestCollision(const Circle* circle, const Polygon* poly, const Transform& transA, const Transform& transB)
	{
		Vec2 center1 = circle->Center + transA.P;
		const uint32 size2 = poly->Count;
		Vec2 points2[LP_POINT_SIZE];
		Vec2  center2{ 0.0f };
		Mat2x2 rb = transB.R.GetMatrix();
		for (uint32 i = 0; i < size2; i++)
		{
			points2[i] = rb * poly->Points[i] + transB.P;
			center2 += points2[i];
		}
		center2 /= (float)size2;
		bool overlap = GJK(center1, center2, [&points2, &size2, &center1, radius = circle->Radius](Vec2 dir)->Vec2 {
			Vec2 point1;
			point1 = dir.Normalize() * radius + center1;
			Vec2 ndir = -dir;
			float Max = points2[0].Dot(ndir);
			uint32 Maxj = 0;
			for (uint32 j = 1; j < size2; j++)
			{
				float value = points2[j].Dot(ndir);
				if (value > Max)
				{
					Maxj = j;
					Max = value;
				}
			}
			return point1 - points2[Maxj];
			});
		return overlap;
	}

	bool LP_API TestCollision(const Box* box, const Polygon* poly, const Transform& transA, const Transform& transB)
	{
		uint32 size1 = 4;
		const uint32 size2 = poly->Count;
		Vec2 points1[4];
		Vec2 points2[LP_POINT_SIZE];
		Vec2  center1 = transA.P + box->Center;
		Vec2  center2{ 0.0f };
		Mat2x2 ra = transA.R.GetMatrix();
		points1[0] = ra * Vec2{  box->Size.x,  box->Size.y } + center1;
		points1[1] = ra * Vec2{  box->Size.x, -box->Size.y } + center1;
		points1[2] = ra * Vec2{ -box->Size.x,  box->Size.y } + center1;
		points1[3] = ra * Vec2{ -box->Size.x, -box->Size.y } + center1;
		Mat2x2 rb = transB.R.GetMatrix();
		for (uint32 i = 0; i < size2; i++)
		{
			points2[i] = rb * poly->Points[i] + transB.P;
			center2 += points2[i];
		}
		center2 /= size2;

		//auto Support = 
		bool overlap = GJK(center1, center2, [&points1, &points2, &size1, &size2](Vec2 dir)->Vec2 {
			float Max = points1[0].Dot(dir);
			uint32 Maxi = 0;
			for (uint32 i = 1; i < size1; i++)
			{
				float value = points1[i].Dot(dir);
				if (value > Max)
				{
					Maxi = i;
					Max = value;
				}
			}
			Vec2 ndir = -dir;
			Max = points2[0].Dot(ndir);
			uint32 Maxj = 0;
			for (uint32 j = 1; j < size2; j++)
			{
				float value = points2[j].Dot(ndir);
				if (value > Max)
				{
					Maxj = j;
					Max = value;
				}
			}
			return points1[Maxi] - points2[Maxj];
			});
		return overlap;

	}

	bool LP_API TestCollision(const Polygon* poly1, const Polygon* poly2, const Transform& transA, const Transform& transB)
	{
		const uint32 size1 = poly1->Count;
		const uint32 size2 = poly2->Count;
		Vec2* points1 = new Vec2[LP_POINT_SIZE];
		Vec2* points2 = new Vec2[LP_POINT_SIZE];
		Vec2  center1{ 0.0f };
		Vec2  center2{ 0.0f };
		Mat2x2 ra = transA.R.GetMatrix();
		for (uint32 i = 0; i < size1; i++)
		{
			points1[i] = ra * poly1->Points[i] + transA.P;
			center1 += points1[i];
		}
		center1 /= float(size1);
		Mat2x2 rb = transB.R.GetMatrix();
		for (uint32 i = 0; i < size2; i++)
		{
			points2[i] = rb * poly2->Points[i] + transB.P;
			center2 += points2[i];
		}
		center2 /= float(size2);

		//auto Support = 
		bool overlap = GJK(center1, center2, [&points1, &points2, &size1, &size2](Vec2 dir)->Vec2 {
			float Max = points1[0].Dot(dir);
uint32 Maxi = 0;
for (uint32 i = 1; i < size1; i++)
{
	float value = points1[i].Dot(dir);
	if (value > Max)
	{
		Maxi = i;
		Max = value;
	}
}
Vec2 ndir = -dir;
Max = points2[0].Dot(ndir);
uint32 Maxj = 0;
for (uint32 j = 1; j < size2; j++)
{
	float value = points2[j].Dot(ndir);
	if (value > Max)
	{
		Maxj = j;
		Max = value;
	}
}
return points1[Maxi] - points2[Maxj];
			});
			return overlap;
	}

	// Generate contact info

	// normal from circleA
	bool LP_API FindCollision(ContactInfo* info, const Circle* circleA, const Circle* circleB, const Transform& transA, const Transform& transB)
	{
		Vec2 dist = circleB->Center + transB.P - circleA->Center - transA.P;
		float r = circleA->Radius + circleB->Radius;
		info->Depths[0] = r - dist.Length();
		info->Normal = dist.Normalize();
		info->Points[0] = circleA->Center + transA.P + info->Normal * circleA->Radius;
		info->RefPoints[0].x = circleA->Radius;
		info->RefPoints[1].x = circleB->Radius;
		info->Count = 1;
		info->Type = CONTACT_TYPE::CIRCLES;
		info->Key.Feature.Type = CONTACT_COMBINATION::CIRCLE_CIRCLE;
		info->Key.Feature.Edge1 = 0;
		info->Key.Feature.Edge2 = 0;
		info->Key.Feature.Order = 0;
		return info->Depths[0] > 0.0f;
	}
	// normal from circle
	bool LP_API FindCollision(ContactInfo* info, const Circle* circle, const Box* box, const Transform& transA, const Transform& transB)
	{
		Vec2 c = (-transB.R).GetMatrix() * (circle->Center + transA.P - transB.P - box->Center);
		Vec2 max = box->Size;
		Vec2 min = -box->Size;
		Vec2 p{ 0.0f, 0.0f };
		p.x = std::max(std::min(c.x, max.x), min.x);
		p.y = std::max(std::min(c.y, max.y), min.y);
		//Vec2 dist = (c - p).Normalize();
		float d1 = std::min(fabs(max.x - c.x), fabs(c.x - min.x));
		float d2 = std::min(fabs(max.y - c.y), fabs(c.y - min.y));
		if (d1 < d2)
		{
			info->RefPoints[0] = { max.x, min.y };
			info->RefPoints[1] = { max.x, max.y };
		}
		else
		{
			info->RefPoints[0] = { max.x, max.y };
			info->RefPoints[1] = { min.x, max.y };
		}
		if (c == p)
		{
			if (d1 < d2)
			{
				info->Normal = Vec2{ 1.0f, 0.0f };
				info->Depths[0] = d1 + circle->Radius;
			}
			else
			{
				info->Normal = Vec2{ 0.0f, 1.0f };
				info->Depths[0] = d2 + circle->Radius;
			}
			if (c.Dot(info->Normal) > 0.0f)
			{
				info->Normal *= -1.0f;
			}
			else
			{
				info->RefPoints[0] *= -1.0f;
				info->RefPoints[1] *= -1.0f;
			}
			info->Normal = (transB.R).GetMatrix() * info->Normal;
		}
		else
		{
			Vec2 normal = (p - c).Normalize();
			if (fabs(normal.Dot(Vec2{ 1.0f, 0.0f })) > fabs(normal.Dot(Vec2{ 0.0f, 1.0f })))
			{
				info->RefPoints[0] = { max.x, min.y };
				info->RefPoints[1] = { max.x, max.y };
				if (normal.Dot(Vec2{ 1.0f, 0.0f }) > 0.0f)
				{
					info->RefPoints[0] = { min.x, max.y };
					info->RefPoints[1] = { min.x, min.y };
				}
			}
			else
			{
				info->RefPoints[0] = { max.x, max.y };
				info->RefPoints[1] = { min.x, max.y };
				if (normal.Dot(Vec2{ 0.0f, 1.0f }) > 0.0f)
				{
					info->RefPoints[1] = { max.x, min.y };
					info->RefPoints[0] = { min.x, min.y };
				}
			}
			info->Normal = (transB.R).GetMatrix() * normal;
			info->Depths[0] = circle->Radius - (p - c).Length();
		}
		info->Points[0] = circle->Center + transA.P + info->Normal * circle->Radius;
		info->Type = CONTACT_TYPE::EDGE_B;
		info->Count = 1;
		info->Key.Feature.Type = CONTACT_COMBINATION::CIRCLE_BOX;
		info->Key.Feature.Edge1 = 0;
		info->Key.Feature.Edge2 = 0;
		info->Key.Feature.Order = 0;
		return info->Depths[0] > 0;
	}
	static inline void BestPoint(Vec2* edge, Vec2* points, uint32 size, const Vec2& n)
	{
		float max = n.Dot(points[0]);
		uint32 maxi = 0;;
		for (int i = 1; i < size; i++)
		{
			float projection = n.Dot(points[i]);
			if (projection > max)
			{
				max = projection;
				maxi = i;
			}
		}
		edge[0] = points[maxi];
		Vec2 v1 = points[(maxi + 1) % size];
		Vec2 v0 = points[(maxi - 1 + size) % size];
		Vec2 l = edge[0] - v1;
		Vec2 r = edge[0] - v0;
		l = l.Normalize();
		r = r.Normalize();
		if (r.Dot(n) <= l.Dot(n))
		{
			edge[1] = v0;
			edge[2] = edge[0];
		}
		else
		{
			edge[1] = edge[0];
			edge[2] = v1;
		}
	}
	static inline void BestPoint(Vec2* edge, Vec2* points, uint32 size, const Vec2& n, uint8 key[2])
	{
		float max = n.Dot(points[0]);
		uint32 maxi = 0;;
		for (int i = 1; i < size; i++)
		{
			float projection = n.Dot(points[i]);
			if (projection > max)
			{
				max = projection;
				maxi = i;
			}
		}
		edge[0] = points[maxi];
		uint32 maxip = (maxi - 1 + size) % size;
		uint32 maxin = (maxi + 1) % size;
		Vec2 v1 = points[maxin];
		Vec2 v0 = points[maxip];
		Vec2 l = edge[0] - v1;
		Vec2 r = edge[0] - v0;
		l = l.Normalize();
		r = r.Normalize();
		if (r.Dot(n) <= l.Dot(n))
		{
			edge[1] = v0;
			edge[2] = edge[0];
			key[0] = maxip;
			key[1] = maxi;
		}
		else
		{
			edge[1] = edge[0];
			edge[2] = v1;
			key[0] = maxi;
			key[1] = maxin;
		}
	}
	static inline bool Clip(ContactInfo* info, Vec2 e1[3], Vec2 e2[3])
	{
		// Find the reference edge
		Vec2* ref, * inc;
		bool flip = false;
		Vec2& n = info->Normal;
		if (fabs((e1[2] - e1[1]).Dot(n)) <= fabs((e2[2] - e2[1]).Dot(n)))
		{
			ref = e1;
			inc = e2;
		}
		else
		{
			ref = e2;
			inc = e1;
			flip = true;
		}
		// clip incident edge using ref[1]
		Vec2 refv = ref[2] - ref[1];
		refv = refv.Normalize();
		float o1 = refv.Dot(ref[1]);
		Vec2 cp[2];
		uint32 cpSize = 0;
		float d1 = refv.Dot(inc[1]) - o1;
		float d2 = refv.Dot(inc[2]) - o1;
		if (d1 >= 0.0f) cp[cpSize++] = inc[1];
		if (d2 >= 0.0f) cp[cpSize++] = inc[2];
		if (d1 * d2 < 0.0f)
		{
			Vec2 e = inc[2] - inc[1];
			float u = d1 / (d1 - d2);
			cp[cpSize++] = inc[1] + e * u;
		}
		if (cpSize < 2) return false;
		// clip cp using ref[2]
		cpSize = 0;
		float o2 = refv.Dot(ref[2]);
		refv *= -1.0f;
		Vec2 cp0 = cp[0];
		Vec2 cp1 = cp[1];
		float t1 = refv.Dot(cp0) + o2;
		float t2 = refv.Dot(cp1) + o2;
		if (t1 >= 0.0f) cp[cpSize++] = cp0;
		if (t2 >= 0.0f) cp[cpSize++] = cp1;
		if (t1 * t2 < 0.0f)
		{
			Vec2 e = cp1 - cp0;
			float u = t1 / (t1 - t2);
			cp[cpSize++] = cp0 + e * u;
		}
		if (cpSize < 2) return false;
		Vec2 refNorm = { refv.y, -refv.x };
		//if (!flip) refNorm *= -1.0f;
		float max = refNorm.Dot(ref[0]);
		float depth0 = refNorm.Dot(cp[0]) - max;
		float depth1 = refNorm.Dot(cp[1]) - max;
		if (depth0 < 0.0f)
		{
			cp[0] = cp[1];
			depth0 = depth1;
			cpSize = 1;
		}
		else if (depth1 < 0.0f)
		{
			cpSize = 1;
		}
		info->Points[0] = cp[0];
		info->Points[1] = cp[1];
		info->Depths[0] = depth0;
		info->Depths[1] = depth1;
		info->Count = cpSize;
		if (flip)
			info->Type = CONTACT_TYPE::EDGE_A;
		else
			info->Type = CONTACT_TYPE::EDGE_B;
		return true;
	};
	static inline bool Clip(ContactInfo* info, Vec2 e1[3], Vec2 e2[3], uint8 key1[2], uint8 key2[2])
	{
		// Find the reference edge
		Vec2* ref, * inc;
		bool flip = false;
		Vec2& n = info->Normal;
		if (fabs((e1[2] - e1[1]).Dot(n)) <= fabs((e2[2] - e2[1]).Dot(n)))
		{
			ref = e1;
			inc = e2;
		}
		else
		{
			ref = e2;
			inc = e1;
			flip = true;
		}
		// clip incident edge using ref[1]
		Vec2 refv = ref[2] - ref[1];
		refv = refv.Normalize();
		float o1 = refv.Dot(ref[1]);
		Vec2 cp[2];
		uint32 cpSize = 0;
		float d1 = refv.Dot(inc[1]) - o1;
		float d2 = refv.Dot(inc[2]) - o1;
		if (d1 >= 0.0f) cp[cpSize++] = inc[1];
		if (d2 >= 0.0f) cp[cpSize++] = inc[2];
		if (d1 * d2 < 0.0f)
		{
			Vec2 e = inc[2] - inc[1];
			float u = d1 / (d1 - d2);
			cp[cpSize++] = inc[1] + e * u;
		}
		if (cpSize < 2) return false;
		// clip cp using ref[2]
		cpSize = 0;
		float o2 = refv.Dot(ref[2]);
		refv *= -1.0f;
		Vec2 cp0 = cp[0];
		Vec2 cp1 = cp[1];
		float t1 = refv.Dot(cp0) + o2;
		float t2 = refv.Dot(cp1) + o2;
		if (t1 >= 0.0f) cp[cpSize++] = cp0;
		if (t2 >= 0.0f) cp[cpSize++] = cp1;
		if (t1 * t2 < 0.0f)
		{
			Vec2 e = cp1 - cp0;
			float u = t1 / (t1 - t2);
			cp[cpSize++] = cp0 + e * u;
		}
		if (cpSize < 2) return false;
		Vec2 refNorm = { refv.y, -refv.x };
		//if (!flip) refNorm *= -1.0f;
		// Adjust depth
		float max = refNorm.Dot(ref[0]);
		float depth0 = refNorm.Dot(cp[0]) - max;
		float depth1 = refNorm.Dot(cp[1]) - max;
		if (depth0 < 0.0f)
		{
			cp[0] = cp[1];
			depth0 = depth1;
			cpSize = 1;
		}
		else if (depth1 < 0.0f)
		{
			cpSize = 1;
		}
		info->Points[0] = cp[0];
		info->Points[1] = cp[1];
		info->Depths[0] = depth0;
		info->Depths[1] = depth1;
		info->Key.Feature.Edge1 = key1[0];
		info->Key.Feature.Edge1 = key2[0];
		info->Key.Feature.Order = 0;
		info->Count = cpSize;
		info->RefPoints[0] = ref[1];
		info->RefPoints[1] = ref[2];
		if (!flip)
			info->Type = CONTACT_TYPE::EDGE_A;
		else
			info->Type = CONTACT_TYPE::EDGE_B;
		return true;
	};
	static inline bool GetContactInfo(ContactInfo* info, Vec2* points1, uint32 size1, Vec2* points2, uint32 size2, const Vec2& n)
	{
		Vec2 e1[3];
		Vec2 e2[3];
		uint8 key1[2];
		uint8 key2[2];
		BestPoint(e1, points1, size1, n, key1);
		BestPoint(e2, points2, size2, -n, key2);
		return Clip(info, e1, e2, key1, key2);
	}
	bool LP_API FindCollision(ContactInfo* info, const Box* box1, const Box* box2, const Transform& transA, const Transform& transB)
	{
		Vec2 axis[4];
		axis[0] = transA.R.Axis();
		axis[1] = transB.R.Axis();
		axis[2] = transA.R.AxisPer();
		axis[3] = transB.R.AxisPer();
		Vec2 center1 = box1->Center + transA.P;
		Vec2 center2 = box2->Center + transB.P;
		Vec2 dist = center2 - center1;
		Mat2x2 ra = transA.R.GetMatrix();
		Mat2x2 rb = transB.R.GetMatrix();
		Mat2x2 raInv = (-transA.R).GetMatrix();
		Mat2x2 rbInv = (-transB.R).GetMatrix();
		Vec2 extend[4];
		extend[0] = ra * box1->Size;
		extend[1] = ra * Vec2{ box1->Size.x, -box1->Size.y };
		extend[2] = rb * box2->Size;
		extend[3] = rb * Vec2{ box2->Size.x, -box2->Size.y };
		float minOverlap = HUGE_VALF;
		uint32 minIndex = 0;
		bool edge2edge = false;

		for (int32 i = 0; i < 4; i++)
		{
			float prod[4];
			prod[0] = fabs(axis[i].Dot(extend[0]));
			prod[1] = fabs(axis[i].Dot(extend[1]));
			prod[2] = fabs(axis[i].Dot(extend[2]));
			prod[3] = fabs(axis[i].Dot(extend[3]));
			float a = std::max(prod[0], prod[1]);
			float b = std::max(prod[2], prod[3]);
			float c = fabs(axis[i].Dot(dist));
			float overlap = a + b - c;
			if (overlap < 0)
				return false;
			if (overlap < minOverlap)
			{
				minIndex = i;
				minOverlap = overlap;
			}
		}
		info->Depths[0] = minOverlap;
		info->Normal = axis[minIndex].Dot(dist) > 0.0f ? axis[minIndex] : -axis[minIndex];
		// CCW
		// Generate points
		Vec2 points1[4];
		points1[0] = center1 + extend[0];
		points1[1] = center1 - extend[1];
		points1[2] = center1 - extend[0];
		points1[3] = center1 + extend[1];
		Vec2 points2[4];
		points2[0] = center2 + extend[2];
		points2[1] = center2 - extend[3];
		points2[2] = center2 - extend[2];
		points2[3] = center2 + extend[3];
		Vec2 e1[3];
		Vec2 e2[3];
		// Find the best points alone normal
		// BestPoint(e1, points1, 4, info->Normal);
		// BestPoint(e2, points2, 4,-info->Normal);
		// bool overlap = Clip(info, e1, e2);
		bool overlap = GetContactInfo(info, points1, 4, points2, 4, info->Normal);
		if (info->Type == CONTACT_TYPE::EDGE_A)
		{
			info->RefPoints[0] = raInv * (info->RefPoints[0] - center1);
			info->RefPoints[1] = raInv * (info->RefPoints[1] - center1);
		}
		else
		{
			info->RefPoints[0] = rbInv * (info->RefPoints[0] - center2);
			info->RefPoints[1] = rbInv * (info->RefPoints[1] - center2);
		}
		info->Key.Feature.Type = CONTACT_COMBINATION::BOX_BOX;
		return overlap;
	}
	bool LP_API FindCollision(ContactInfo* info, const Circle* circle, const Polygon* poly, const Transform& transA, const Transform& transB)
	{
		Vec2 center1 = circle->Center + transA.P;
		const uint32 size2 = poly->Count;
		Vec2 points2[LP_POINT_SIZE];
		Vec2  center2{ 0.0f };
		Mat2x2 rb = transB.R.GetMatrix();
		for (uint32 i = 0; i < size2; i++)
		{
			points2[i] = rb * poly->Points[i] + transB.P;
			center2 += points2[i];
		}
		center2 /= size2;
		auto SupportA = [&center1, radius = circle->Radius](Vec2 dir, Vec2* p1, Vec2* p2)->int32 {
			Vec2 point1;
			point1 = dir.Normalize() * radius + center1;
			*p1 = point1;
			return 1;
		};
		auto SupportB = [&points2, &size2](Vec2 dir, Vec2* p1, Vec2* p2)->int32 {
			float Max = points2[0].Dot(dir);
			uint32 Maxj[2] = { 0, 0 };
			int32 count = 0;
			for (uint32 j = 1; j < size2; j++)
			{
				float value = points2[j].Dot(dir);
				if (value > Max)
				{
					count = 1;
					Maxj[0] = j;
					Max = value;
				}
				else if (value == Max)
				{
					count = 2;
					Maxj[1] = j;
				}
			}
			*p1 = points2[Maxj[0]];
			*p2 = points2[Maxj[1]];
			return count;
		};
		bool overlap = GJKwithEPA(info, center1, center2, SupportA, SupportB);

		Vec2 p1 = points2[0];
		Vec2 p2 = points2[1];
		Vec2 p21 = (p2 - p1).Normalize();
		float Max = Vec2(p21.y, -p21.x).Dot(-info->Normal);
		uint32 BestP[2];
		BestP[0] = 0;
		BestP[1] = 1;
		for (uint32 j = 1; j < size2; j++)
		{
			p1 = points2[j];
			p2 = points2[(j + 1) % size2];
			p21 = (p2 - p1).Normalize();
			float mm = Vec2(p21.y, -p21.x).Dot(-info->Normal);
			if (mm > Max)
			{
				BestP[0] = j;
				BestP[1] = (j + 1) % size2;
				Max = mm;
			}
		}
		p1 = points2[BestP[0]];
		p2 = points2[BestP[1]];
		Mat2x2 rbInv = (-transB.R).GetMatrix();
		info->RefPoints[0] = rbInv * (p1 - transB.P);
		info->RefPoints[1] = rbInv * (p2 - transB.P);
		info->Type = CONTACT_TYPE::EDGE_B;
		info->Count = 1;
		info->Points[0] = center1 + info->Normal * circle->Radius;
		info->Key.Feature.Type = CONTACT_COMBINATION::CIRCLE_POLYGON;
		info->Key.Feature.Edge1 = 0;
		info->Key.Feature.Edge2 = 0;
		info->Key.Feature.Order = 0;
		return overlap;
	}
	bool LP_API FindCollision(ContactInfo* info, const Box* box, const Polygon* poly, const Transform& transA, const Transform& transB)
	{
		const uint32 size1 = 4;
		const uint32 size2 = poly->Count;
		Vec2 points1[4];
		Vec2 points2[LP_POINT_SIZE];
		Vec2  center1 = transA.P + box->Center;
		Vec2  center2{ 0.0f };
		Mat2x2 ra = transA.R.GetMatrix();
		points1[0] = ra * Vec2{ box->Size.x,  box->Size.y } + center1;
		points1[1] = ra * Vec2{ -box->Size.x,  box->Size.y } + center1;
		points1[2] = ra * Vec2{ -box->Size.x, -box->Size.y } + center1;
		points1[3] = ra * Vec2{ box->Size.x, -box->Size.y } + center1;
		Mat2x2 rb = transB.R.GetMatrix();
		for (uint32 i = 0; i < size2; i++)
		{
			points2[i] = rb * poly->Points[i] + transB.P;
			center2 += points2[i];
		}
		center2 /= size2;

		auto SupportA = [&points1, &size1](Vec2 dir, Vec2* p1, Vec2* p2)->int32 {
			float Max = points1[0].Dot(dir);
			uint32 Maxi[2] = { 0, 0 };
			int32 count = 1;
			for (uint32 i = 1; i < size1; i++)
			{
				float value = points1[i].Dot(dir);
				if (value > Max)
				{
					count = 1;
					Maxi[0] = i;
					Max = value;
				}
				else if (value == Max)
				{
					count = 2;
					Maxi[1] = i;
				}
			}
			*p1 = points1[Maxi[0]];
			*p2 = points1[Maxi[1]];
			return count;
		};
		auto SupportB = [&points2, &size2](Vec2 dir, Vec2* p1, Vec2* p2)->int32 {
			float Max = points2[0].Dot(dir);
			uint32 Maxj[2] = { 0, 1 };
			int32 count = 1;
			for (uint32 j = 1; j < size2; j++)
			{
				float value = points2[j].Dot(dir);
				if (value > Max)
				{
					count = 1;
					Maxj[0] = j;
					Max = value;
				}
				else if (value == Max)
				{
					count = 2;
					Maxj[1] = j;
				}
			}
			*p1 = points2[Maxj[0]];
			*p2 = points2[Maxj[1]];
			return count;
		};
		bool overlap = GJKwithEPA(info, center1, center2, SupportA, SupportB);
		if (!overlap) return false;
		//Vec2 e1[3];
		//Vec2 e2[3];
		//// Find the best points alone normal
		//BestPoint(e1, points1, size1, info->Normal);
		//BestPoint(e2, points2, size2, -info->Normal);
		//// Find the reference edge
		//overlap = Clip(info, e1, e2);
		overlap = GetContactInfo(info, points1, size1, points2, size2, info->Normal);
		Mat2x2 raInv = (-transA.R).GetMatrix();
		Mat2x2 rbInv = (-transB.R).GetMatrix();
		if (info->Type == CONTACT_TYPE::EDGE_A)
		{
			info->RefPoints[0] = raInv * (info->RefPoints[0] - center1);
			info->RefPoints[1] = raInv * (info->RefPoints[1] - center1);
		}
		else
		{
			info->RefPoints[0] = rbInv * (info->RefPoints[0] - transB.P);
			info->RefPoints[1] = rbInv * (info->RefPoints[1] - transB.P);
		}
		info->Key.Feature.Type = CONTACT_COMBINATION::BOX_POLYGON;
		return overlap;
	}
	bool LP_API FindCollision(ContactInfo* info, const Polygon* poly1, const Polygon* poly2, const Transform& transA, const Transform& transB)
	{
		const uint32 size1 = poly1->Count;
		const uint32 size2 = poly2->Count;
		Vec2 points1[LP_POINT_SIZE];
		Vec2 points2[LP_POINT_SIZE];
		Vec2  center1{ 0.0f };
		Vec2  center2{ 0.0f };
		Mat2x2 ra = transA.R.GetMatrix();
		for (uint32 i = 0; i < size1; i++)
		{
			points1[i] = ra * poly1->Points[i] + transA.P;
			center1 += points1[i];
		}
		center1 /= float(size1);
		Mat2x2 rb = transB.R.GetMatrix();
		for (uint32 i = 0; i < size2; i++)
		{
			points2[i] = rb * poly2->Points[i] + transB.P;
			center2 += points2[i];
		}
		center2 /= float(size2);

		auto SupportA = [&points1, &size1](Vec2 dir, Vec2* p1, Vec2* p2)->int32 {
			float Max = points1[0].Dot(dir);
			uint32 Maxi[2] = { 0, 0 };
			int32 count = 1;
			for (uint32 i = 1; i < size1; i++)
			{
				float value = points1[i].Dot(dir);
				if (value > Max)
				{
					count = 1;
					Maxi[0] = i;
					Max = value;
				}
				else if (value == Max)
				{
					count = 2;
					Maxi[1] = i;
				}
			}
			*p1 = points1[Maxi[0]];
			*p2 = points1[Maxi[1]];
			return count;
		};
		auto SupportB = [&points2, &size2](Vec2 dir, Vec2* p1, Vec2* p2)->int32{
			float Max = points2[0].Dot(dir);
			uint32 Maxj[2] = { 0, 1 };
			int32 count = 1;
			for (uint32 j = 1; j < size2; j++)
			{
				float value = points2[j].Dot(dir);
				if (value > Max)
				{
					count = 1;
					Maxj[0] = j;
					Max = value;
				}
				else if (value == Max)
				{
					count = 2;
					Maxj[1] = j;
				}
			}
			*p1 = points2[Maxj[0]];
			*p2 = points2[Maxj[1]];
			return count;
		};
		bool overlap = GJKwithEPA(info, center1, center2, SupportA, SupportB);
		if (!overlap) return false;
		//Vec2 e1[3];
		//Vec2 e2[3];
		//// Find the best points alone normal
		//BestPoint(e1, points1, size1, info->Normal);
		//BestPoint(e2, points2, size2, -info->Normal);
		//// Find the reference edge
		//overlap = Clip(info, e1, e2);
		overlap = GetContactInfo(info, points1, size1, points2, size2, info->Normal);
		Mat2x2 raInv = (-transA.R).GetMatrix();
		Mat2x2 rbInv = (-transB.R).GetMatrix();
		if (info->Type == CONTACT_TYPE::EDGE_A)
		{
			info->RefPoints[0] = raInv * (info->RefPoints[0] - transA.P);
			info->RefPoints[1] = raInv * (info->RefPoints[1] - transA.P);
		}
		else
		{
			info->RefPoints[0] = rbInv * (info->RefPoints[0] - transB.P);
			info->RefPoints[1] = rbInv * (info->RefPoints[1] - transB.P);
		}
		info->Key.Feature.Type = CONTACT_COMBINATION::POLYGON_POLYGON;
		return overlap;
	}
}

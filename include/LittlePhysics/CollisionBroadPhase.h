#pragma once
#include "Core.h"
#include "Body.h"
#include <vector>
#include <list>

#define MAX_NODES 4000

namespace LP
{
	struct LP_API CollisionPair
	{
		Body* body1;
		Body* body2;
	};

	struct LP_API DbvhNode
	{
		int32 Child[2];
		int32 Parent;
		uint32 ChildIndex = 0;

		AABB AaBb;
		float Area;

		Body* body;
		bool Updated = false;
	};

	class LP_API DbvhTree
	{
	public:
		using Index = int32;
		#define IndexNull -1
		DbvhTree() = default;
		void TestCollision();
		Index Update(Index handle, const AABB& aabb);
		Index Insert(Body* body, const  AABB& aabb);
		void Remove(Index handle);
		CollisionPair* GetCollisionPairs()
		{
			return m_CollisionPairs.data();
		}
		uint32 GetCollisionPairsCount() const
		{
			return m_CollisionPairs.size();
		}
	private:
		float Area(const AABB& aabb)
		{
			return (aabb.Max.x - aabb.Min.x) * (aabb.Max.y - aabb.Min.y);
		}
		float Area(const AABB& aabb1, const AABB& aabb2)
		{
			float maxX, maxY, minX, minY;
			maxX = fmaxf(aabb1.Max.x, aabb2.Max.x);
			maxY = fmaxf(aabb1.Max.y, aabb2.Max.y);
			minX = fminf(aabb1.Min.x, aabb2.Min.x);
			minY = fminf(aabb1.Min.y, aabb2.Min.y);
			return (maxX - minX) * (maxY - minY);
		}
		AABB Union(const AABB& aabb1, const AABB& aabb2)
		{
			AABB aabb;
			aabb.Max.x = fmaxf(aabb1.Max.x, aabb2.Max.x);
			aabb.Max.y = fmaxf(aabb1.Max.y, aabb2.Max.y);
			aabb.Min.x = fminf(aabb1.Min.x, aabb2.Min.x);
			aabb.Min.y = fminf(aabb1.Min.y, aabb2.Min.y);
			return aabb;
		}
		//void RecycleNode(Index index);
		void RefitFrom(Index index);
		Index AllocateNode(DbvhNode* node);
		DbvhNode& AllocateNode(Index& index);
		void FreeNode(Index index);
		void TestCollision(Index index);
		void TestCollision2(Index indexA, Index indexB);
	public:
		Index						m_Root = -1;
		uint32						m_NodeCount = 0; 
		std::vector<CollisionPair> m_CollisionPairs;
		DbvhNode					m_Nodes[MAX_NODES];
		std::list<Index>			m_FreeNodes;
		const float					m_EnlargeFactor = 1.2f;
	};
}
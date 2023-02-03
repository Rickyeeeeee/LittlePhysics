#pragma once
#include "Core.h"
#include "CollisionNarrowPhase.h"
#include "CollisionBroadPhase.h"
#include "Contact.h"
#include <vector>

namespace LP {

	/*class LP_API CollisionManager
	{
	public:
		
		CollisionManager() = default;

		uint32 AddBody(Body* body);

		void RemoveBody(uint32 handle);

		void Update();

		void Collide();

		Contact* GetContactData()
		{
			return m_Contacts.data();
		}
		uint32 GetContectCount() const
		{
			return m_Contacts.size();
		}

	private:
		std::vector <CollisionPair> m_CollisionPairs;
		std::vector <Contact>		m_Contacts;
		DbvhTree					m_DbvhTree;
	};*/
}
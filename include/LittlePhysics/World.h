#pragma once
#include "Core.h"
#include "Math.h"
#include "DataTypes.h"
#include "Body.h"
#include "CollisionNarrowPhase.h"
#include "CollisionBroadPhase.h"
#include "Constraint.h"
#include "Contact.h"
#include <functional>
#include <vector>

// TODO: Change to unlimited version
#define MAX_BODY 4000
#define MAX_CONSTRAINT 50000

namespace LP {
	class LP_API World
	{
	public:
		World();
		Body* CreateBody(BodyCreateInfo* info);
		void DeleteBody(Body* body);
		void StepImpulse(float dt);
		void Step(float dt);
		uint32 GetBodyCount() const
		{
			return m_BodyCount;
		}

		// Might be deleted
		const std::vector<ContactDebug>& GetContacts() const
		{
			return m_ContactDebugs;
		}
		const DbvhTree& GetDbvhTree() const
		{
			return m_DbvhTree;
		}
		uint32 GetContactCount() const
		{
			return m_ContactCount;
		}
		// Might be deleted
		bool& GetSleep()
		{
			return m_EnableSleeping;
		}
	private:
		void Initialize();
		void Collide();
		void InitializeVelocityConstraints();
		void InitializePositionConstraints();
		void WarmStart();
		void SolveVelocityConstraints(float dt);
		void SolvePositionConstraints(float dt);
	private:
		using Dispatcher 
			= std::function<bool(LP::ContactInfo* info, LP::Shape* shapeA, LP::Shape* shapeB, 
							const LP::Transform& tranA, const LP::Transform& tranB)>;

		std::vector<ContactDebug>	m_ContactDebugs;
		Dispatcher				FindCollision[3][3];
		DbvhTree				m_DbvhTree;
		Body*					m_BodyList = nullptr;
		uint32					m_BodyCount = 0;
		// For time stepping
		Position				m_Positions[MAX_BODY];
		Vec2					m_dPositions[MAX_BODY];
		Velocity				m_Velocities[MAX_BODY];
		Body*					m_Bodies[MAX_BODY];

		
		Contact					m_ContactConstraints[MAX_CONSTRAINT];
		uint32					m_ContactCount;
		Contact*				m_Contacts = nullptr;
		bool					m_Sleeping = false;
		bool					m_EnableSleeping = true;
		uint32					m_SleepTime = 0;
	};
}
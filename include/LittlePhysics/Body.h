#pragma once
#include "Core.h"
#include "DataTypes.h"
#include "Shape.h"
#include "Math.h"

namespace LP {

	struct ContactEdge;

	struct LP_API Position
	{
		Vec2 c;
		float a;
	};

	struct LP_API Velocity
	{
		Vec2 v;
		float w;
	};

	enum class BODY_TYPE
	{
		STATIC = 0, DYNAMIC, KINEMATIC
	};

	struct LP_API BodyCreateInfo
	{
		BODY_TYPE				BodyType;
		float					Density;
		float					Restitution = 0.5f;
		float					Friction = 0.0f;
	};

	class LP_API Body
	{
	public:

		Body()
		{
			m_Shape = new Box;
			m_ShapeType = COLLISION_SHAPE_TYPE::BOX;
			m_Type = BODY_TYPE::DYNAMIC;
			((Box*)m_Shape)->Center = { 0.0f, 0.0f };
			((Box*)m_Shape)->Size = { 0.5f, 0.5f };
			Area = 1.0f;
			M = 1.0f;
			Minv = 1.0f;
			I = 1.0f / 12.0f;
			Iinv = 12.0f;
		}

		Body(BodyCreateInfo* info);
		

		void AttachCircleShape(float r);

		void AttachBoxShape(const Vec2& size);

		//void AttachPolygonShape(const std::vector<Vec2>& vertices);
		void AttachPolygonShape(const Vec2* points, uint32 size);
		
		void ApplyForce(const Vec2& force);

		void SetPosition(Vec2 position)
		{
			m_Tranf.P = position;
		}

		Vec2 GetPosition() const
		{
			return m_Tranf.P;
		}

		void SetRotation(float radians)
		{
			m_Tranf.R.Set(radians);
		}

		float GetRotation() const
		{
			return m_Tranf.R.GetAngle();
		}

		void SetVelocity(const Vec2& v)
		{
			V = v;
		}

		void SetType(BODY_TYPE type)
		{
			m_Type = type;
		}

		BODY_TYPE GetType() const
		{
			return m_Type;
		}

		Transform GetTransform() const
		{
			return m_Tranf;
		}

		COLLISION_SHAPE_TYPE GetShape(Shape*& shape) const
		{
			shape = m_Shape;
			return m_ShapeType;
		}
	private:
		friend class World;
		friend class NormalConstraint;
		friend class FrictionConstraint;

		Shape* m_Shape = nullptr;

		COLLISION_SHAPE_TYPE	m_ShapeType;
		BODY_TYPE				m_Type;

		float Area;
		float m_Density = 1.0f;
		float m_Restituion = 1.0f;
		float m_Friction = 1.0f;

		// Internal Data //
		LP::Vec2 V = { 0.0f };
		float W = 0.0f;
		LP::Vec2 F = { 0.0f };
		float T = 0.0f;

		float M = 1.0f;
		float Minv = 1.0f;
		float I = 1.0f;
		float Iinv = 1.0f;

		uint32 m_ID;
		int32 m_CollisionHandle = -1;
		////////////////////////
		//Position m_Position;
		//Velocity m_Velocity;

		Transform m_Tranf;
		Body* m_Prev = nullptr;
		Body* m_Next = nullptr;
		// Keep a reference of contacts, doesn't allocate memory
		//Contact* m_Contacts = nullptr;
		ContactEdge* m_ContactEdges;
	};

}
#include <LittlePhysics/World.h>
#include <iostream>

namespace LP {

	World::World()
	{
		m_Contacts = nullptr;
		FindCollision[0][0] = [](LP::ContactInfo* info, LP::Shape* shapeA, LP::Shape* shapeB,
			const LP::Transform& tranA, const LP::Transform& tranB)->bool {
				bool v = LP::FindCollision(info, (LP::Circle*)shapeA, (LP::Circle*)shapeB, tranA, tranB);
				return v;
		};
		FindCollision[0][1] = [](LP::ContactInfo* info, LP::Shape* shapeA, LP::Shape* shapeB,
			const LP::Transform& tranA, const LP::Transform& tranB)->bool {
				bool v = LP::FindCollision(info, (LP::Circle*)shapeA, (LP::Box*)shapeB, tranA, tranB);
				return v;
		};
		FindCollision[0][2] = [](LP::ContactInfo* info, LP::Shape* shapeA, LP::Shape* shapeB,
			const LP::Transform& tranA, const LP::Transform& tranB)->bool {
				bool v = LP::FindCollision(info, (LP::Circle*)shapeA, (LP::Polygon*)shapeB, tranA, tranB);
				return v;
		};
		FindCollision[1][0] = [](LP::ContactInfo* info, LP::Shape* shapeA, LP::Shape* shapeB,
			const LP::Transform& tranA, const LP::Transform& tranB)->bool {
				bool v = LP::FindCollision(info, (LP::Circle*)shapeB, (LP::Box*)shapeA, tranB, tranA);
				info->Normal *= -1.0f;
				return v;
		};
		FindCollision[1][1] = [](LP::ContactInfo* info, LP::Shape* shapeA, LP::Shape* shapeB,
			const LP::Transform& tranA, const LP::Transform& tranB)->bool {
				bool v = LP::FindCollision(info, (LP::Box*)shapeA, (LP::Box*)shapeB, tranA, tranB);
				return v;
		};
		FindCollision[1][2] = [](LP::ContactInfo* info, LP::Shape* shapeA, LP::Shape* shapeB,
			const LP::Transform& tranA, const LP::Transform& tranB)->bool {
				bool v = LP::FindCollision(info, (LP::Box*)shapeA, (LP::Polygon*)shapeB, tranA, tranB);
				return v;
		};
		FindCollision[2][0] = [](LP::ContactInfo* info, LP::Shape* shapeA, LP::Shape* shapeB,
			const LP::Transform& tranA, const LP::Transform& tranB)->bool {
				bool v = LP::FindCollision(info, (LP::Circle*)shapeB, (LP::Polygon*)shapeA, tranB, tranA);
				info->Normal *= -1.0f;
				return v;
		};
		FindCollision[2][1] = [](LP::ContactInfo* info, LP::Shape* shapeA, LP::Shape* shapeB,
			const LP::Transform& tranA, const LP::Transform& tranB)->bool {
				bool v = LP::FindCollision(info, (LP::Box*)shapeB, (LP::Polygon*)shapeA, tranB, tranA);
				info->Normal *= -1.0f;
				return v;
		};
		FindCollision[2][2] = [](LP::ContactInfo* info, LP::Shape* shapeA, LP::Shape* shapeB,
			const LP::Transform& tranA, const LP::Transform& tranB)->bool {
				bool v = LP::FindCollision(info, (LP::Polygon*)shapeB, (LP::Polygon*)shapeA, tranB, tranA);
				info->Normal *= -1.0f;
				return v;
		};
	}

	Body* World::CreateBody(BodyCreateInfo* info)
	{

		Body* body = new Body(info);
		if (!m_BodyList)
		{
			body->m_Prev = nullptr;
			body->m_Next = nullptr;
			m_BodyList = body;
		}
		else
		{
			body->m_Prev = nullptr;
			body->m_Next = m_BodyList;
			m_BodyList->m_Prev = body;
			m_BodyList = body;
		}
		m_BodyCount++;
		return body;
	}

	void World::DeleteBody(Body* body)
	{
		if (!body) return;
		m_DbvhTree.Remove(body->m_CollisionHandle);

		ContactEdge* ce = body->m_ContactEdges;
		while (ce)
		{
			auto* next = ce->Next;

			Contact* contact = ce->Contact;

			if (contact->m_Prev)
				contact->m_Prev->m_Next = contact->m_Next;
			else
				m_Contacts = contact->m_Next;

			if (contact->m_Next)
				contact->m_Next->m_Prev = contact->m_Prev;

			auto* ce1 = &contact->ContactEdge1;
			auto* body1 = contact->body1;
			if (ce1->Prev)
				ce1->Prev->Next = ce1->Next;
			else
				body1->m_ContactEdges = ce1->Next;
			if (ce1->Next)
				ce1->Next->Prev = ce1->Prev;

			auto* ce2 = &contact->ContactEdge2;
			auto* body2 = contact->body2;
			if (ce2->Prev)
				ce2->Prev->Next = ce2->Next;
			else
				body2->m_ContactEdges = ce2->Next;
			if (ce2->Next)
				ce2->Next->Prev = ce2->Prev;

			// TODO: Change allocator
			delete contact;

			ce = next;
		}

		Body* prev = body->m_Prev;
		Body* next = body->m_Next;
		if (prev)
		{
			prev->m_Next = next;
			if (next)
			{
				next->m_Prev = prev;
			}
		}
		else
		{
			m_BodyList = next;
			if (next)
			{
				next->m_Prev = nullptr;
			}
		}
		m_BodyCount--;
		delete body;
	}

	void World::Step(float dt)
	{
		Initialize();
		Collide();

		// Apply forces and copy data
		for (uint32 i = 0; i < m_BodyCount; i++)
		{
			Body* body = m_Bodies[i];
			if (body->m_Type == BODY_TYPE::DYNAMIC)
			{
				body->F += Vec2{ 0.0f,-9.8f } *body->M;
			}

			m_Positions[i].c = body->m_Tranf.R.GetAngle();
			m_Positions[i].c = body->m_Tranf.P;
			m_Positions[i].a = body->m_Tranf.R.GetAngle();

			m_Velocities[i].v = body->V + body->F * body->Minv * dt;
			m_Velocities[i].w = body->W + body->T * body->Iinv * dt;
		}


		InitializeVelocityConstraints();

		WarmStart();
		for (uint32 iter = 0; iter < 10; iter++)
		{
			SolveVelocityConstraints(dt);
		}

		// Apply velocities
		for (uint32 i = 0; i < m_BodyCount; i++)
		{
			m_Positions[i].c = m_Positions[i].c + m_Velocities[i].v * dt;
			m_Positions[i].a = m_Positions[i].a + m_Velocities[i].w * dt * 1.0f;
		}

		// Copy back data;
		for (uint32 i = 0; i < m_BodyCount; i++)
		{
			Body* body = m_Bodies[i];
			body->m_Tranf.P = m_Positions[i].c;
			body->m_Tranf.R.Set(m_Positions[i].a);
			body->V = m_Velocities[i].v;
			body->W = m_Velocities[i].w;
			body->F = { 0.0f, 0.0f };
			body->T = 0.0f;
			body = body->m_Next;
		}
	}

	void World::Initialize()
	{
		Body* body = m_BodyList;
		uint32 i = 0;
		while (body)
		{
			m_Bodies[i] = body;
			if (body->m_CollisionHandle < 0 && body->m_Shape != nullptr)
			{
				body->m_CollisionHandle = m_DbvhTree.Insert(body, body->m_Shape->GetAABB(body->m_Tranf));
			}
			body->m_ID = i;
			body = body->m_Next;
			i++;
		}
		m_BodyCount = i;
	}

	void World::Collide()
	{
		m_ContactDebugs.clear();

		// Use Broad phase
		for (uint32 i = 0; i < m_BodyCount; i++)
		{
			Body* body = m_Bodies[i];
			Shape* shape;
			body->GetShape(shape);
			if (body->m_CollisionHandle != IndexNull)
				body->m_CollisionHandle = m_DbvhTree.Update(body->m_CollisionHandle, shape->GetAABB(body->m_Tranf));
		}
		m_DbvhTree.TestCollision();
		uint32 collisionPairCount = m_DbvhTree.GetCollisionPairsCount();
		CollisionPair* collisionPairs = m_DbvhTree.GetCollisionPairs();

		m_ContactCount = 0;

		for (uint32 i = 0; i < collisionPairCount; i++)
		{
			Body* body1 = collisionPairs[i].body1;
			Body* body2 = collisionPairs[i].body2;

			ContactInfo info;
			if (body1->m_Type == BODY_TYPE::STATIC && body2->m_Type == BODY_TYPE::STATIC)
				continue;
			if (!body1->m_Shape || !body2->m_Shape)
				continue;
			ContactEdge* ce = body1->m_ContactEdges;
			bool found = false;
			while (ce)
			{
				if (ce->Other == body2)
				{
					found = true;
					break;
				}
				ce = ce->Next;
			}
			if (!found)
			{
				// Insert Contact
				Contact* contact = new Contact;
				contact->count = 100;
				contact->cID.ID = 0xffffffff;
				contact->body1 = body1;
				contact->body2 = body2;
				contact->m_Prev = nullptr;
				contact->m_Next = m_Contacts;
				if (m_Contacts)
					m_Contacts->m_Prev = contact;
				m_Contacts = contact;

				// insert on body1
				auto* ce1 = &contact->ContactEdge1;
				ce1->Contact = contact;
				ce1->Other = body2;
				ce1->Prev = nullptr;
				ce1->Next = body1->m_ContactEdges;
				if (body1->m_ContactEdges)
				{
					body1->m_ContactEdges->Prev = ce1;
				}
				body1->m_ContactEdges = ce1;

				// insert on body2
				auto* ce2 = &contact->ContactEdge2;
				ce2->Contact = contact;
				ce2->Other = body1;
				ce2->Prev = nullptr;
				ce2->Next = body2->m_ContactEdges;
				if (body2->m_ContactEdges)
				{
					body2->m_ContactEdges->Prev = ce2;
				}
				body2->m_ContactEdges = ce2;
			}
		}

		Contact* contact = m_Contacts;
		while (contact)
		{

			Body* body1 = contact->body1;
			Body* body2 = contact->body2;
			contact->index1 = body1->m_ID;
			contact->index2 = body2->m_ID;
			Contact* nextContact = contact->m_Next;

			ContactInfo info;
			bool collision = FindCollision[static_cast<uint32>(body1->m_ShapeType)][static_cast<uint32>(body2->m_ShapeType)](&info,
				body1->m_Shape, body2->m_Shape, body1->m_Tranf, body2->m_Tranf);
			if (collision)
			{
				// TODO: Find edges for warm starting
				// Adding Constraint
				// contact->body1 = body1->m_ID;
				// contact->body2 = body2->m_ID;
				contact->normal = info.Normal.Normalize();
				uint8 oldCount = contact->count;
				if (info.Type == CONTACT_TYPE::EDGE_EDGE)
					contact->count = 2;
				else
					contact->count = 1;
#define WARM_START 1
#if WARM_START
				if (info.Key.ID == contact->cID.ID && contact->count == oldCount)
				{
				}
				else
				{
					for (uint8 i = 0; i < 2; i++)
					{
						contact->cc[i].nc.impulse = 0.0f;
						contact->cc[i].fc.impulse = 0.0f;
					}
				}
#else
				for (uint32 i = 0; i < 2; i++)
				{
					contact->cc[i].nc.impulse = 0.0f;
					contact->cc[i].fc.impulse = 0.0f;
				}
#endif
				contact->cID = info.Key;

				for (uint8 i = 0; i < contact->count; i++)
				{
					contact->cc[i].depth = info.Depths[i];
					contact->cc[i].r1 = info.Points[i] - body1->GetPosition();
					contact->cc[i].r2 = info.Points[i] - body2->GetPosition();
				}

				ContactDebug c;
				c.BodyA = body1;
				c.BodyB = body2;
				c.info = info;
				m_ContactDebugs.push_back(c);
			}
			// Remove the contact
			else
			{
				if (contact->m_Prev)
					contact->m_Prev->m_Next = contact->m_Next;
				else
					m_Contacts = contact->m_Next;

				if (contact->m_Next)
					contact->m_Next->m_Prev = contact->m_Prev;

				auto* ce1 = &contact->ContactEdge1;
				if (ce1->Prev)
					ce1->Prev->Next = ce1->Next;
				else
					body1->m_ContactEdges = ce1->Next;
				if (ce1->Next)
					ce1->Next->Prev = ce1->Prev;

				auto* ce2 = &contact->ContactEdge2;
				if (ce2->Prev)
					ce2->Prev->Next = ce2->Next;
				else
					body2->m_ContactEdges = ce2->Next;
				if (ce2->Next)
					ce2->Next->Prev = ce2->Prev;

				// TODO: Change allocator
				delete contact;
			}
			contact = nextContact;
		}
	}

	void World::InitializeVelocityConstraints()
	{
		for (Contact* c = m_Contacts; c; c = c->m_Next)
		{
			//auto cc = m_ContactConstraints[i];
			Body* body1 = c->body1;
			Body* body2 = c->body2;
			c->m1 = body1->Minv;
			c->i1 = body1->Iinv;
			c->m2 = body2->Minv;
			c->i2 = body2->Iinv;
			if (body1->m_Type == BODY_TYPE::STATIC)
			{
				c->m1 = 0.0f;
				c->i1 = 0.0f;
			}
			if (body2->m_Type == BODY_TYPE::STATIC)
			{
				c->m2 = 0.0f;
				c->i2 = 0.0f;
			}
			for (uint8 i = 0; i < c->count; i++)
			{
				auto& cc = c->cc[i];
				// friction constraint
				auto& fc = cc.fc;
				//fc.impulse = 0.0f;
				Vec2 u = { -c->normal.y, c->normal.x };
				fc.c1 = -u;
				fc.a1 = -(cc.r1.Cross(u));
				fc.c2 = u;
				fc.a2 = (cc.r2.Cross(u));
				fc.mc = 1.0f /
					(fc.c1.Length2() * c->m1 + fc.a1 * fc.a1 * c->i1
						+ fc.c2.Length2() * c->m2 + fc.a2 * fc.a2 * c->i2);
				// normal constraint
				auto& nc = cc.nc;
				// TODO: Warm starting
				//nc.impulse = 0.0f;
				Vec2 n = c->normal;
				nc.c1 = -n;
				nc.a1 = -cc.r1.Cross(n);
				nc.c2 = n;
				nc.a2 = cc.r2.Cross(n);
				nc.mc = 1.0f /
					(nc.c1.Length2() * c->m1 + nc.a1 * nc.a1 * c->i1
						+ nc.c2.Length2() * c->m2 + nc.a2 * nc.a2 * c->i2);
				auto [v1, w1] = m_Velocities[c->index1];
				auto [v2, w2] = m_Velocities[c->index2];
				float vRel = n.Dot(v2 + cc.r2.Cross(w2) - v1 - cc.r1.Cross(w1));
				nc.bias = -0.1f * vRel;
				//m_ContactConstraints[i] = cc;
			}
		}
	}

	void World::WarmStart()
	{
#if WARM_START


		for (Contact* c = m_Contacts; c; c = c->m_Next)
		{
			//auto cc = m_ContactConstraints[i];
			auto& cc = *c;
			cc.cc[0].fc.impulse *= 0.6f;
			cc.cc[0].nc.impulse *= 0.6f;
			cc.cc[1].fc.impulse *= 0.6f;
			cc.cc[1].nc.impulse *= 0.6f;
			uint32 index1 = c->index1;
			uint32 index2 = c->index2;
			Velocity v1 = m_Velocities[index1];
			Velocity v2 = m_Velocities[index2];
			float m1 = cc.m1;
			float i1 = cc.i1;
			float m2 = cc.m2;
			float i2 = cc.i2;

			for (uint8 i = 0; i < c->count; i++)
			{
				auto nc = c->cc[i].nc;
				auto fc = c->cc[i].fc;
				float depth = c->cc[i].depth;
				float lambda = nc.impulse;

				v1.v = v1.v + nc.c1 * lambda * m1;
				v1.w = v1.w + nc.a1 * lambda * i1;
				v2.v = v2.v + nc.c2 * lambda * m2;
				v2.w = v2.w + nc.a2 * lambda * i2;

				lambda = fc.impulse;

				v1.v = v1.v + fc.c1 * lambda * m1;
				v1.w = v1.w + fc.a1 * lambda * i1;
				v2.v = v2.v + fc.c2 * lambda * m2;
				v2.w = v2.w + fc.a2 * lambda * i2;

				//c->cc[i].nc.impulse = 0.0f;
				//c->cc[i].fc.impulse = 0.0f;
			}
			m_Velocities[index1] = v1;
			m_Velocities[index2] = v2;

		}
#endif // 
	}

	void World::SolveVelocityConstraints(float dt)
	{
		float dtinv = 1.0f / dt;
		for (Contact* c = m_Contacts; c; c = c->m_Next)
		{
			//auto cc = m_ContactConstraints[i];
			auto& cc = *c;
			uint32 index1 = c->index1;
			uint32 index2 = c->index2;
			float m1 = cc.m1;
			float i1 = cc.i1;
			float m2 = cc.m2;
			Velocity v1 = m_Velocities[index1];
			Velocity v2 = m_Velocities[index2];
			float i2 = cc.i2;

			for (uint8 i = 0; i < c->count; i++)
			{
				auto& nc = c->cc[i].nc;
				auto& fc = c->cc[i].fc;
				float depth = c->cc[i].depth;
				float lambda;
				float friction;
				float newImpulse;

				lambda = -nc.mc * (nc.c1.Dot(v1.v) + nc.a1 * v1.w + nc.c2.Dot(v2.v) + nc.a2 * v2.w - (depth - 0.01f) * 0.5f * dtinv - nc.bias);
				newImpulse = fmaxf(0.0f, nc.impulse + lambda);
				lambda = newImpulse - nc.impulse;
				nc.impulse = newImpulse;

				v1.v = v1.v + nc.c1 * lambda * m1;
				v1.w = v1.w + nc.a1 * lambda * i1;
				v2.v = v2.v + nc.c2 * lambda * m2;
				v2.w = v2.w + nc.a2 * lambda * i2;

				lambda = -fc.mc * (fc.c1.Dot(v1.v) + fc.a1 * v1.w + fc.c2.Dot(v2.v) + fc.a2 * v2.w);
				//friction = nc.impulse * 2.0f;
				friction = 1000.0f;
				newImpulse = fmaxf(-friction, fminf(fc.impulse + lambda, friction));
				lambda = newImpulse - fc.impulse;
				fc.impulse = newImpulse;

				//

				v1.v = v1.v + fc.c1 * lambda * m1;
				v1.w = v1.w + fc.a1 * lambda * i1;
				v2.v = v2.v + fc.c2 * lambda * m2;
				v2.w = v2.w + fc.a2 * lambda * i2;


			}


			m_Velocities[index1] = v1;
			m_Velocities[index2] = v2;

		}
	}
}

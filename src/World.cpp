#include <LittlePhysics/World.h>
#include <iostream>

namespace LP {

	static inline void FlipContactInfo(ContactInfo* info)
	{
		info->Normal *= -1.0f;
		CONTACT_TYPE type = CONTACT_TYPE::EDGE_B;
		if (info->Type == CONTACT_TYPE::EDGE_B)
			type = CONTACT_TYPE::EDGE_A;
		if (info->Type != CONTACT_TYPE::CIRCLES)
			info->Type = type;
	}

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
				FlipContactInfo(info);
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
				FlipContactInfo(info);
				return v;
		};
		FindCollision[2][1] = [](LP::ContactInfo* info, LP::Shape* shapeA, LP::Shape* shapeB,
			const LP::Transform& tranA, const LP::Transform& tranB)->bool {
				bool v = LP::FindCollision(info, (LP::Box*)shapeB, (LP::Polygon*)shapeA, tranB, tranA);
				FlipContactInfo(info);
				return v;
		};
		FindCollision[2][2] = [](LP::ContactInfo* info, LP::Shape* shapeA, LP::Shape* shapeB,
			const LP::Transform& tranA, const LP::Transform& tranB)->bool {
				bool v = LP::FindCollision(info, (LP::Polygon*)shapeB, (LP::Polygon*)shapeA, tranB, tranA);
				FlipContactInfo(info);
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
		m_Sleeping = false;
		return body;
	}

	void World::DeleteBody(Body* body)
	{
		if (!body) return;
		m_DbvhTree.Remove(body->m_CollisionHandle);
		m_Sleeping = false;
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
		float linearTolerance = 0.05f;
		float angularTolerance = 0.1f;
		Initialize();
		Collide();
		// Apply forces and copy data
		Vec2 gravity = { 0.0f, -98.0f };
		//gravity = 0.0f;
		for (uint32 i = 0; i < m_BodyCount; i++)
		{
			Body* body = m_Bodies[i];
			if (body->F != Vec2{ 0.0f, 0.0f })
				m_Sleeping = false;
			if (m_Sleeping && body->V.Dot(body->V) > linearTolerance * linearTolerance)
				m_Sleeping = false;
			if (m_Sleeping && body->W * body->W > angularTolerance * angularTolerance)
				m_Sleeping = false;
		}
		if (m_Sleeping)
			m_SleepTime++;
		else
			m_SleepTime = 0;
		if (m_EnableSleeping && m_Sleeping && m_SleepTime > 20)
		{
			for (uint32 i = 0; i < m_BodyCount; i++)
			{
				m_Velocities[i].v = 0.0f;
				m_Velocities[i].w = 0.0f;
			}
			/*for (Contact* c = m_Contacts; c; c = c->m_Next)
			{
				for (uint32 i = 0; i < c->count; i++)
				{
					c->vc[i].nc.impulse = 0.0f;
					c->vc[i].fc.impulse = 0.0f;
				}
			}*/
			return;
		}
		for (uint32 i = 0; i < m_BodyCount; i++)
		{
			Body* body = m_Bodies[i];
			if (body->m_Type == BODY_TYPE::DYNAMIC)
			{
				body->F += gravity * body->M;
			}

			m_Positions[i].c = body->m_Tranf.P;
			m_Positions[i].a = body->m_Tranf.R.GetAngle();
			auto p1 = m_Positions[i];
			if (p1.c != p1.c)
				int a = 3;
			m_Velocities[i].v = body->V + body->F * body->Minv * dt;
			if (!body->m_FixRotation)
				m_Velocities[i].w = body->W + body->T * body->Iinv * dt;
			else
				m_Velocities[i].w = 0.0f;
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
			auto [v, w] = m_Velocities[i];
			Body* body = m_Bodies[i];
			/*if (v.Dot(v) < linearTolerance * linearTolerance)
				m_Velocities[i].v = 0.0f;
			if (w * w < angularTolerance * angularTolerance)
				m_Velocities[i].w = 0.0f;*/

			m_Positions[i].c = m_Positions[i].c + m_Velocities[i].v * dt;
			//if (!body->m_FixRotation)
				m_Positions[i].a = m_Positions[i].a + m_Velocities[i].w * dt;
		}

		for (uint32 iter = 0; iter < 6; iter++)
		{
			SolvePositionConstraints(dt);
		}
		// Copy back data;
		m_Sleeping = true;	
		uint32 sleepCount = 0;
		for (uint32 i = 0; i < m_BodyCount; i++)
		{
			Body* body = m_Bodies[i];
			body->m_Tranf.P = m_Positions[i].c;
			body->m_Tranf.R.Set(m_Positions[i].a);
			body->V = m_Velocities[i].v;
			body->W = m_Velocities[i].w;
			uint32 sleep = 1;
			if (body->V.Dot(body->V) > linearTolerance * linearTolerance)
			{
				m_Sleeping = false;
				sleep = 0;
			}
			if (body->W * body->W > angularTolerance * angularTolerance)
			{
				sleep = 0;
				m_Sleeping = false;	
			}

			sleepCount += sleep;

			body->F = { 0.0f, 0.0f };
			body->T = 0.0f;
			body = body->m_Next;


		}
		//std::cout << (sleepCount) << "/" << (m_BodyCount) << "\n";
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
				m_Sleeping = false;
			}
			body->m_ID = i;
			if (body->m_Tranf.P.x != body->m_Tranf.P.x)
				int a = 9;
			if (body->m_Tranf.P.y != body->m_Tranf.P.y)
				int a = 9;
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
		uint32 warmStartCount = 0;
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
				for (uint32 i = 0; i < info.Count; i++)
					if (info.Type == CONTACT_TYPE::EDGE_B)
						contact->Points[i] = body1->m_Tranf.Reverse(info.Points[i]);
					else
						contact->Points[i] = body2->m_Tranf.Reverse(info.Points[i]);
				contact->type = info.Type;
				contact->localPoints[0] = info.RefPoints[0];
				contact->localPoints[1] = info.RefPoints[1];
				contact->normal = info.Normal;
				uint32 oldCount = contact->count;
				contact->count = info.Count;
				if (info.Key.ID == contact->cID.ID && contact->count == oldCount)
				{
					warmStartCount++;
				}
				else
				{
					for (uint32 i = 0; i < 2; i++)
					{
						contact->vc[i].nc.impulse = 0.0f;
						contact->vc[i].fc.impulse = 0.0f;
					}
				}
				contact->cID = info.Key;

				for (uint32 i = 0; i < contact->count; i++)
				{
					contact->vc[i].depth = info.Depths[i];
					contact->vc[i].r1 = info.Points[i] - body1->GetPosition();
					contact->vc[i].r2 = info.Points[i] - body2->GetPosition();
				}

				ContactDebug c;
				c.BodyA = body1;
				c.BodyB = body2;
				c.info = info;
				m_ContactDebugs.push_back(c);
			}
			else
			{
			// Remove the contact
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
		//std::cout << warmStartCount << std::endl;
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
			if (body1->m_FixRotation)
				c->i1 = 0.0f;
			if (body2->m_FixRotation)
				c->i2 = 0.0f;
			for (uint32 i = 0; i < c->count; i++)
			{
				auto& cc = c->vc[i];
				// Position constraint
				auto& pc = c->pc[i];
				pc.r1 = body1->m_Tranf.R.Inverse().GetMatrix() * cc.r1;
				pc.r2 = body2->m_Tranf.R.Inverse().GetMatrix() * cc.r2;

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
				float mixR = (body1->m_Restituion + body2->m_Restituion) * 0.5f;
				nc.bias = 0.0f;
				float threshold = fmin(-1.0f, -10.0f * mixR);
				if (vRel < threshold)
					nc.bias = -mixR * vRel;
				//nc.bias = -0.5f * vRel;
				//m_ContactConstraints[i] = cc;
			}
			// Prepare block solver
			if (c->count == 2)
			{
				auto nc1 = c->vc[0].nc;
				auto nc2 = c->vc[1].nc;
				float m1 = c->m1;
				float m2 = c->m2;
				float i1 = c->i1;
				float i2 = c->i2;

				c->mc[0][0] = nc1.c1.Length2()	* m1 + nc1.a1 * nc1.a1 * i1 + nc1.c2.Length2()		* m2 + nc1.a2 * nc1.a2 * i2;
				c->mc[0][1] = nc1.c1.Dot(nc2.c1) * m1 + nc1.a1 * nc2.a1 * i1 + nc1.c2.Dot(nc2.c2)	* m2 + nc1.a2 * nc2.a2 * i2;
				c->mc[1][0] = nc2.c1.Dot(nc1.c1) * m1 + nc2.a1 * nc1.a1 * i1 + nc2.c2.Dot(nc1.c2)	* m2 + nc2.a2 * nc1.a2 * i2;
				c->mc[1][1] = nc2.c1.Length2()	* m1 + nc2.a1 * nc2.a1 * i1 + nc2.c2.Length2()		* m2 + nc2.a2 * nc2.a2 * i2;

				const Mat2x2& mc = c->mc;

				float det = mc[0][0] * mc[1][1] - mc[0][1] * mc[1][0];
				if (mc[0][0] * mc[0][0] < 1000.0f * det)
				{
					det = 1.0f / det;
					c->mcInv.Ex = {  mc[1][1] * det, -mc[0][1] * det };
					c->mcInv.Ey = { -mc[1][0] * det,  mc[0][0] * det };
				}
				else
				{
					c->count = 1;
				}
			}
		}
	}

	void World::InitializePositionConstraints()
	{
	}

	void World::WarmStart()
	{
		for (Contact* c = m_Contacts; c; c = c->m_Next)
		{
			//auto cc = m_ContactConstraints[i];
			auto& cc = *c;
			/*cc.cc[0].fc.impulse *= 0.6f;
			cc.cc[0].nc.impulse *= 0.6f;
			cc.cc[1].fc.impulse *= 0.6f;
			cc.cc[1].nc.impulse *= 0.6f;*/
			uint32 index1 = c->index1;
			uint32 index2 = c->index2;
			Velocity v1 = m_Velocities[index1];
			Velocity v2 = m_Velocities[index2];
			float m1 = cc.m1;
			float i1 = cc.i1;
			float m2 = cc.m2;
			float i2 = cc.i2;

			for (uint32 i = 0; i < c->count; i++)
			{
				auto nc = c->vc[i].nc;
				auto fc = c->vc[i].fc;
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


			}
			m_Velocities[index1] = v1;
			m_Velocities[index2] = v2;

		}
	}

	struct LP_API PositionManifold
	{
		PositionManifold(Contact* contact, const Transform& tranfA, const Transform& tranfB, uint32 index)
		{
			switch (contact->type)
			{
			case CONTACT_TYPE::CIRCLES:
			{
				normal = (tranfB.P - tranfA.P).Normalize();
				float ra1 = contact->localPoints[0].x;
				float ra2 = contact->localPoints[1].x;
				depth = ra1 + ra2 - (tranfB.P - tranfA.P).Length();
				r1 = normal * ra1;
				r2 = -normal * (ra2 - depth);
			}
				break;
			case CONTACT_TYPE::EDGE_A:
			{

				Vec2 point = tranfB * contact->Points[index];
				Vec2 edge = tranfA.R.GetMatrix() * (contact->localPoints[1] - contact->localPoints[0]).Normalize();
				normal = Vec2{ edge.y, -edge.x };
				r1 = point - tranfA.P;
				r2 = point - tranfB.P;
				depth = (tranfA * contact->localPoints[0]).Dot(normal) - point.Dot(normal);
			}
				break;
			case CONTACT_TYPE::EDGE_B:
			{

				Vec2 point = tranfA * contact->Points[index];
				Vec2 edge = tranfB.R.GetMatrix() * (contact->localPoints[1] - contact->localPoints[0]).Normalize();
				normal = Vec2{ edge.y, -edge.x };
				r1 = point - tranfA.P;
				r2 = point - tranfB.P;
				depth = (tranfB * contact->localPoints[0]).Dot(normal) - point.Dot(normal);
				normal = -normal;
			}
				break;
			}
			//r1 = tranfA * contact->vc[index].
		}
		float depth;
		Vec2 normal;
		Vec2 r1;
		Vec2 r2;
	};

	void World::SolvePositionConstraints(float dt)
	{
		const float depthError = 0.05f;
		const float Bumer = 0.2f;
		const float maxBumer = 0.2f;
		const float minBumer = -0.0f;
		for (Contact* c = m_Contacts; c; c = c->m_Next)
		{
			auto& cc = *c;
			uint32 index1 = c->index1;
			uint32 index2 = c->index2;
			Body* body1 = m_Bodies[index1];
			Body* body2 = m_Bodies[index2];
			float m1 = cc.m1;
			float i1 = cc.i1;
			float m2 = cc.m2;
			float i2 = cc.i2;
			Position p1 = m_Positions[index1];
			Position p2 = m_Positions[index2];
			for (uint32 i = 0; i < c->count; i++)
			{
				Transform tranfA;
 				tranfA.R.Set(p1.a);
				tranfA.P = p1.c;
				Transform tranfB;
				tranfB.R.Set(p2.a);
				tranfB.P = p2.c;
				PositionManifold pm(c, tranfA, tranfB, i);
				Vec2 normal = pm.normal;
				float seperation = pm.depth;
				float rn1 = pm.r1.Cross(normal);
				float rn2 = pm.r2.Cross(normal);
				float mc = m1 + m2 + i1 * rn1 * rn1 + i2 * rn2 * rn2;
				float lambda = fminf(fmaxf(Bumer * (seperation - depthError), minBumer), maxBumer);
				if (mc <= 0.0f)
					lambda = 0.0f;
				else
					lambda = lambda / mc;
				Vec2 P = normal * lambda;
				float pcAdjust = 1.0f;
				float pcBdjust = 1.0f;
				if (c->count == 1)
				{
					pcAdjust = 1.0f;
					pcBdjust = 1.0f;
				}
				p1.c -= P * m1 * pcAdjust;
				p1.a -= pm.r1.Cross(P) * i1 / pcBdjust;
				p2.c += P * m2 * pcAdjust;
				p2.a += pm.r2.Cross(P) * i2 / pcBdjust;
				// TODO: block positoin solver
			}
			m_Positions[index1] = p1;
			m_Positions[index2] = p2;
		}
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
			float i2 = cc.i2;
			Velocity v1 = m_Velocities[index1];
			Velocity v2 = m_Velocities[index2];
			Body* body1 = m_Bodies[index1];
			Body* body2 = m_Bodies[index2];
			for (uint32 i = 0; i < c->count; i++)
			{
				auto& nc = c->vc[i].nc;
				auto& fc = c->vc[i].fc;
				float depth = c->vc[i].depth;
				float lambda;
				float friction;
				float newImpulse;

				lambda = -fc.mc * (fc.c1.Dot(v1.v) + fc.a1 * v1.w + fc.c2.Dot(v2.v) + fc.a2 * v2.w);
				friction = nc.impulse * (body1->m_Friction + body2->m_Friction);
  				//friction = 10000.0f;
				newImpulse = fmaxf(-friction, fminf(fc.impulse + lambda, friction));
				//newImpulse = fmaxf(-friction, fminf(fc.impulse + lambda, friction));
				lambda = newImpulse - fc.impulse;
				fc.impulse = newImpulse;

				v1.v = v1.v + fc.c1 * lambda * m1;
				v1.w = v1.w + fc.a1 * lambda * i1;
				v2.v = v2.v + fc.c2 * lambda * m2;
				v2.w = v2.w + fc.a2 * lambda * i2;
			}

			if (c->count <= 1)
			{
				for (uint32 i = 0; i < c->count; i++)
				{

					auto& nc = c->vc[i].nc;
					auto& fc = c->vc[i].fc;
					float depth = c->vc[i].depth;
					float lambda;
					float friction;
					float newImpulse;

					lambda = -nc.mc * (nc.c1.Dot(v1.v) + nc.a1 * v1.w + nc.c2.Dot(v2.v) + nc.a2 * v2.w - nc.bias);
					//lambda = -nc.mc * (nc.c1.Dot(v1.v) + nc.a1 * v1.w + nc.c2.Dot(v2.v) + nc.a2 * v2.w  - nc.bias);
					//lambda = -nc.mc * (nc.c1.Dot(v1.v) + nc.a1 * v1.w + nc.c2.Dot(v2.v) + nc.a2 * v2.w - (depth - 0.01f) * 0.2f * dtinv - nc.bias);
					newImpulse = fmaxf(0.0f, nc.impulse + lambda);
					//newImpulse = nc.impulse + lambda;
					lambda = newImpulse - nc.impulse;
					nc.impulse = newImpulse;

					v1.v = v1.v + nc.c1 * lambda * m1;
					v1.w = v1.w + nc.a1 * lambda * i1;
					v2.v = v2.v + nc.c2 * lambda * m2;
					v2.w = v2.w + nc.a2 * lambda * i2;
				}
			}
			else
			for  (;;)
			{
				// TODO: Block Solver
				Vec2 lambda;
				auto& nc1 = c->vc[0].nc;
				auto& nc2 = c->vc[1].nc;
				Vec2 a{ nc1.impulse, nc2.impulse };
				Vec2 bias;
				bias[0] = nc1.bias;
				bias[1] = nc2.bias;
				Vec2 B;
				B[0] = nc1.c1.Dot(v1.v) + nc1.a1 * v1.w + nc1.c2.Dot(v2.v) + nc1.a2 * v2.w;
				B[1] = nc2.c1.Dot(v1.v) + nc2.a1 * v1.w + nc2.c2.Dot(v2.v) + nc2.a2 * v2.w;

				float mc12 = (nc2.c1.Dot(nc1.c1) * m1 - nc2.a1 * nc1.a1 * i1 + nc2.c2.Dot(nc1.c2) * m2 + nc2.a2 * nc1.a2 * i2);
				lambda = cc.mcInv * (-B + bias) + a;
				Vec2 vn = { 0.0f, 0.0f };
				if (lambda.x >= 0.0f && lambda.y >= 0.0f)
				{
					nc1.impulse = lambda[0];
					nc2.impulse = lambda[1];
					Vec2 P = lambda - a;
					v1.v += (nc1.c1 * P[0] + nc2.c1 * P[1]) * m1;
					v1.w += (nc1.a1 * P[0] + nc2.a1 * P[1]) * i1;
					v2.v += (nc1.c2 * P[0] + nc2.c2 * P[1]) * m2;
					v2.w += (nc1.a2 * P[0] + nc2.a2 * P[1]) * i2;
					break;
				}

 				lambda.x = 0.0f;
				lambda.y = nc2.mc * (-B[1] + bias[1] + c->mc[1][0] * a.x) + a.y;
				vn[0] = c->mc[0][1] * (lambda.y -a.y) - c->mc[0][0] * a.x + B[0] - bias[0];
				vn[1] = 0.0f;
				//vn[0] = c->mc[0]
				if (lambda.y >= 0.0f && vn[0] >= 0.0f)
				{
					nc1.impulse = lambda[0];
					nc2.impulse = lambda[1];
					Vec2 P = lambda - a;
					v1.v += (nc1.c1 * P[0] + nc2.c1 * P[1]) * m1;
					v1.w += (nc1.a1 * P[0] + nc2.a1 * P[1]) * i1;
					v2.v += (nc1.c2 * P[0] + nc2.c2 * P[1]) * m2;
					v2.w += (nc1.a2 * P[0] + nc2.a2 * P[1]) * i2;
					break;
				}
				
				lambda.x = nc1.mc * (-B[0] + bias[0] + c->mc[0][1] * a.y) + a.x;
				lambda.y = 0.0f;
				vn[0] = 0.0f;
				vn[1] = c->mc[1][0] * (lambda.x - a.x) - c->mc[1][1] * a.y + B[1] - bias[1];

				if (lambda.x >= 0.0f && vn[1] >= 0.0f)
				{
					nc1.impulse = lambda[0];
					nc2.impulse = lambda[1];
					Vec2 P = lambda - a;
					v1.v += (nc1.c1 * P[0] + nc2.c1 * P[1]) * m1;
					v1.w += (nc1.a1 * P[0] + nc2.a1 * P[1]) * i1;
					v2.v += (nc1.c2 * P[0] + nc2.c2 * P[1]) * m2;
					v2.w += (nc1.a2 * P[0] + nc2.a2 * P[1]) * i2;
					break;
				}

				lambda = { 0.0f, 0.0f };
				vn = c->mc * (-a) + B - bias;

				if (vn[0] >= 0.0f && vn[1] >= 0.0f)
				{
					nc1.impulse = lambda[0];
					nc2.impulse = lambda[1];
					Vec2 P = lambda - a;
					v1.v += (nc1.c1 * P[0] + nc2.c1 * P[1]) * m1;
					v1.w += (nc1.a1 * P[0] + nc2.a1 * P[1]) * i1;
					v2.v += (nc1.c2 * P[0] + nc2.c2 * P[1]) * m2;
					v2.w += (nc1.a2 * P[0] + nc2.a2 * P[1]) * i2;
					break;
				}
				break;
			}

			m_Velocities[index1] = v1;
			m_Velocities[index2] = v2;

		}
	}
}


#include <LittlePhysics/Body.h>

void LP::Body::ApplyForce(const Vec2& force)
{
	F += force;
}

static float iratio = 1.0f;

void LP::Body::AttachCircleShape(float r)
{
	if (m_Shape)
		delete m_Shape;
	m_ShapeType = COLLISION_SHAPE_TYPE::CIRCLE;
	m_Shape = new Circle;
	((Circle*)m_Shape)->Radius = r;
	((Circle*)m_Shape)->Center = { 0.0f };

	Area = m_Shape->GetArea();
	M = Area * m_Density;
	Minv = 1.0f / M;
	I = iratio * m_Shape->GetInertia(m_Density);
	Iinv = 1.0f / I;
}

void LP::Body::AttachBoxShape(const Vec2& size)
{
	if (m_Shape)
		delete m_Shape;
	m_ShapeType = COLLISION_SHAPE_TYPE::BOX;
	m_Shape = new Box;
	((Box*)m_Shape)->Size = size;
	((Box*)m_Shape)->Center = { 0.0f };

	Area = m_Shape->GetArea();
	M = Area * m_Density;
	Minv = 1.0f / M;
	I = iratio * m_Shape->GetInertia(m_Density);
	Iinv = 1.0f / I;
}

void LP::Body::AttachPolygonShape(const Vec2* points, uint32 size)
{
	if (m_Shape)
		delete m_Shape;
	m_ShapeType = COLLISION_SHAPE_TYPE::POLYGON;
	m_Shape = new Polygon;
	dynamic_cast<Polygon*>(m_Shape)->Count = size;
	for (uint32 i = 0; i < size; i++)
	{
		dynamic_cast<Polygon*>(m_Shape)->Points[i] = points[i];
	}

	Area = m_Shape->GetArea();
	M = Area * m_Density;
	Minv = 1.0f / M;
	I = iratio * m_Shape->GetInertia(m_Density);
	Iinv = 1.0f / I;
}

LP::Body::Body(BodyCreateInfo* info)
{
	m_Type = info->BodyType;
	m_Density = info->Density;
	m_Restituion = info->Restitution;
	m_Friction = info->Friction;
	M = m_Density * 1.0f;
	Minv = 1.0f / M;
	I = 1.0f;
	Iinv = 1.0f;
	m_ContactEdges = nullptr;
	V = Vec2{ 0.0f };
	W = 0.0f;
}
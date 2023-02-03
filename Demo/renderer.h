#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <LittlePhysics/Shape.h>

class Renderer
{
public:
	struct Camera
	{
		glm::vec2 Position;
		float Zoom;
	};

	static void Init();
	static void Begin(const Camera& camera);
	// General Commands
	static void DrawBox(const glm::vec2& max, const glm::vec2& min, const glm::vec3& color);
	static void DrawPoly(const glm::vec2* points, uint32_t size, const glm::vec3& color);
	static void DrawCircle(const glm::vec2& pos, float radius, const glm::vec3& color, bool fill = true);

	// LP Shapes
	static void DrawCircle(const LP::Circle& circle, const LP::Transform& tran, const glm::vec3& color = glm::vec3{ 1.0f });
	static void DrawBox(const LP::Box& box, const LP::Transform& tran, const glm::vec3& color = glm::vec3{ 1.0f });
	static void DrawAABB(const LP::AABB& aabb, const glm::vec3& color = glm::vec3{ 1.0f });
	static void DrawLine(const LP::Vec2& start, const LP::Vec2& length, const glm::vec3& color = glm::vec3{ 1.0f });
	static void DrawPoly(const LP::Polygon& poly, const LP::Transform& tran, const glm::vec3& color = glm::vec3{ 1.0f });
	static void End();

	// Utilities
	static void EnableBlend();
	static void DisableBlend();


	Renderer() = delete;
};
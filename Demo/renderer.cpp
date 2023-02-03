#include "renderer.h"

#include <glad/glad.h>
#include "utils.h"
#include <vector>

static inline glm::vec2 ToGLM(LP::Vec2 vec)
{
	return { vec.x, vec.y };
}

const char *lineVertexShaderSource = 
R"(	#version 450 core 

	layout (location = 0) in vec2 a_Position;
	layout (location = 1) in vec3 a_Color;

	uniform mat4 u_MVP;
	out vec3 color;

	void main(){
	
	gl_Position = u_MVP * vec4(a_Position, 0.0, 1.0);
	color = a_Color;

	})";

const char* lineFragmentShaderSource =
R"(	#version 450 core 

	layout (location = 0) out vec4 FragColor;

	in vec3 color;

	void main(){
	
		FragColor = vec4(color, 1.0);

	})";

const char* circleVSsource =
R"(
	#version 450 core
	layout (location = 0) in vec2 a_Position;

	uniform vec2 center;
	uniform float zoom;
	uniform float radius;
	out vec2 uv;
	void main()
	{
		uv = a_Position;
		gl_Position = vec4((a_Position * radius + center) * zoom , 0.0, 1.0);
	}
)";

const char* circleFSsource =
R"(
	#version 450 core
	layout (location = 0) out vec4 FragColor;
	uniform float radius;
	uniform vec3 color;
	uniform bool fill;
	in vec2 uv;
	void main()
	{
		float distance = 1.0 - length(uv);
		float v = smoothstep(0.0, 0.01, distance); 
		if (!fill) 
			v *= (1.0 - step(0.05 / radius, distance));
		FragColor = vec4(color * v, v);
	}
)";
const char* QuadVSsource =
R"(	#version 450 core 

	layout (location = 0) in vec2 a_Position;
	layout (location = 1) in vec3 a_Color;

	uniform mat4 u_MVP;
	out vec3 color;

	void main(){
	
	gl_Position = u_MVP * vec4(a_Position, -1.0, 1.0);
	color = a_Color;

	})";
const char* QuadFSsource =
R"(	#version 450 core 

	layout (location = 0) out vec4 FragColor;

	in vec3 color;

	void main(){
	
		FragColor = vec4(color, 0.3);

	})";
struct RendererData
{
	struct Vertex
	{
		glm::vec2 Position;
		glm::vec3 Color;
	};

	std::vector <Vertex> LineVertices;
	std::vector <uint32_t> LineIndices;

	std::vector <Vertex> CircleVertices;
	std::vector <uint32_t> CircleIndices;

	std::vector <Vertex> TriangleVertices;
	std::vector <uint32_t> TriangleIndices;

	GLuint LineVertexBuffer;
	GLuint LineIndexBuffer;
	GLuint LineVertexArray;

	GLuint CircleVertexBuffer;
	GLuint CircleIndexBuffer;
	GLuint CircleVertexArray;

	GLuint TriangleVertexBuffer;
	GLuint TriangleIndexBuffer;
	GLuint TriangleVertexArray;

	const uint32_t MaxVertices = 10000;
	const uint32_t MaxIndices = 10000;

	GLuint LineShaderProgram;
	GLuint CircleShaderProgram;
	GLuint TriangleShaderProgram;
};

static RendererData* s_RendererData;
static GLuint CreateShader(const char* vsSource, const char* fsSource)
{
	GLuint program;
	GLuint vs = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vs, 1, &vsSource, nullptr);
	glCompileShader(vs);

	GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fs, 1, &fsSource, nullptr);
	glCompileShader(fs);

	program = glCreateProgram();
	glAttachShader(program, vs);
	glAttachShader(program, fs);
	glLinkProgram(program);
	glValidateProgram(program);

	glDeleteShader(vs);
	glDeleteShader(fs);

	return program;
}
void Renderer::Init()
{
	glLineWidth(0.5f);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	s_RendererData = new RendererData;

	glGenVertexArrays(1, &s_RendererData->LineVertexArray);
	glBindVertexArray(s_RendererData->LineVertexArray);
	glGenBuffers(1, &s_RendererData->LineVertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, s_RendererData->LineVertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, s_RendererData->MaxVertices * sizeof(RendererData::Vertex), nullptr, GL_DYNAMIC_DRAW);
	glGenBuffers(1, &s_RendererData->LineIndexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, s_RendererData->LineIndexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, s_RendererData->MaxIndices * sizeof(uint32_t), nullptr, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 2, GL_FLOAT, false, sizeof(RendererData::Vertex), (void*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, false, sizeof(RendererData::Vertex), (void*)8);
	glEnableVertexAttribArray(1);
	glBindVertexArray(0);

	glGenVertexArrays(1, &s_RendererData->TriangleVertexArray);
	glBindVertexArray(s_RendererData->TriangleVertexArray);
	glGenBuffers(1, &s_RendererData->TriangleVertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, s_RendererData->TriangleVertexBuffer);
	glBufferData(GL_ARRAY_BUFFER, s_RendererData->MaxVertices * sizeof(RendererData::Vertex), nullptr, GL_DYNAMIC_DRAW);
	glGenBuffers(1, &s_RendererData->TriangleIndexBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, s_RendererData->TriangleIndexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, s_RendererData->MaxIndices * sizeof(uint32_t), nullptr, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(0, 2, GL_FLOAT, false, sizeof(RendererData::Vertex), (void*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, false, sizeof(RendererData::Vertex), (void*)8);
	glEnableVertexAttribArray(1);
	glBindVertexArray(0);

	glGenVertexArrays(1, &s_RendererData->CircleVertexArray);
	glBindVertexArray(s_RendererData->CircleVertexArray);
	glGenBuffers(1, &s_RendererData->CircleVertexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, s_RendererData->CircleVertexBuffer);
	float vertices[] = {
		1.0f, 1.0f, 
		1.0f, -1.0f, 
		-1.0f, -1.0f,
		-1.0f, 1.0f
	};
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
	glGenBuffers(1, &s_RendererData->CircleIndexBuffer);
	uint32_t indices[] = {
		0, 1, 2,
		2, 3, 0
	};
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, s_RendererData->CircleIndexBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 2, GL_FLOAT, false, sizeof(float) * 2, (void*)0);
	glEnableVertexAttribArray(0);
	glBindVertexArray(0);

	s_RendererData->LineShaderProgram = CreateShader(lineVertexShaderSource, lineFragmentShaderSource);
	s_RendererData->CircleShaderProgram = CreateShader(circleVSsource, circleFSsource);
	s_RendererData->TriangleShaderProgram = CreateShader(QuadVSsource, QuadFSsource);
	
}

void Renderer::Begin(const Camera& camera)
{
	glm::mat4 mvp = glm::scale(glm::mat4(1.0f), glm::vec3(camera.Zoom, camera.Zoom, 1.0f)) * glm::translate(glm::mat4(1.0f), glm::vec3(camera.Position, 0.0f));
	//mvp = glm::mat4(1.0f);
	glUseProgram(s_RendererData->LineShaderProgram);
	auto l = glGetUniformLocation(s_RendererData->LineShaderProgram, "u_MVP");
	glUniformMatrix4fv(l, 1, false, glm::value_ptr(mvp));

	glUseProgram(s_RendererData->TriangleShaderProgram);
	l = glGetUniformLocation(s_RendererData->TriangleShaderProgram, "u_MVP");
	glUniformMatrix4fv(l, 1, false, glm::value_ptr(mvp));

	glUseProgram(s_RendererData->CircleShaderProgram);
	glUniform1f(glGetUniformLocation(s_RendererData->CircleShaderProgram, "zoom"), camera.Zoom);
	
	s_RendererData->LineVertices.clear();
	s_RendererData->LineIndices.clear();
	s_RendererData->TriangleVertices.clear();
	s_RendererData->TriangleIndices.clear();
}

void Renderer::DrawBox(const glm::vec2& max, const glm::vec2& min, const glm::vec3& color)
{
	uint32_t offset = s_RendererData->LineVertices.size();
	s_RendererData->LineVertices.emplace_back(RendererData::Vertex{{ max.x, max.y }, color});
	s_RendererData->LineVertices.emplace_back(RendererData::Vertex{{ max.x, min.y }, color});
	s_RendererData->LineVertices.emplace_back(RendererData::Vertex{{ min.x, min.y }, color});
	s_RendererData->LineVertices.emplace_back(RendererData::Vertex{{ min.x, max.y }, color});
	s_RendererData->LineIndices.push_back(offset + 0);
	s_RendererData->LineIndices.push_back(offset + 1);
	s_RendererData->LineIndices.push_back(offset + 1);
	s_RendererData->LineIndices.push_back(offset + 2);
	s_RendererData->LineIndices.push_back(offset + 2);
	s_RendererData->LineIndices.push_back(offset + 3);
	s_RendererData->LineIndices.push_back(offset + 3);
	s_RendererData->LineIndices.push_back(offset + 0);
}

void Renderer::DrawPoly(const glm::vec2* points, uint32_t size, const glm::vec3& color)
{
	uint32_t offset = s_RendererData->LineVertices.size();
	for (uint32_t i = 0; i < size; i++)
	{
		s_RendererData->LineVertices.emplace_back(RendererData::Vertex{ points[i], color });
		s_RendererData->LineIndices.push_back(offset + i);
		s_RendererData->LineIndices.push_back(offset + (i + 1) % (size));
	}
}

void Renderer::DrawCircle(const glm::vec2& pos, float radius, const glm::vec3& color, bool fill)
{
	glUseProgram(s_RendererData->CircleShaderProgram);
	glBindVertexArray(s_RendererData->CircleVertexArray);
	int lc = glGetUniformLocation(s_RendererData->CircleShaderProgram, "color");
	glUniform3fv(lc, 1, glm::value_ptr(color));
	glUniform2fv(glGetUniformLocation(s_RendererData->CircleShaderProgram, "center"), 1, glm::value_ptr(pos));
	glUniform1f(glGetUniformLocation(s_RendererData->CircleShaderProgram, "radius"), radius);
	glUniform1i(glGetUniformLocation(s_RendererData->CircleShaderProgram, "fill"), fill);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}


void Renderer::DrawCircle(const LP::Circle& circle, const LP::Transform& tran, const glm::vec3& color)
{
	glUseProgram(s_RendererData->CircleShaderProgram);
	glBindVertexArray(s_RendererData->CircleVertexArray);
	int lc = glGetUniformLocation(s_RendererData->CircleShaderProgram, "color");
	glUniform3fv(lc, 1, glm::value_ptr(color));
	glUniform2fv(glGetUniformLocation(s_RendererData->CircleShaderProgram, "center"), 1, glm::value_ptr(ToGLM(circle.Center + tran.P)));
	glUniform1f(glGetUniformLocation(s_RendererData->CircleShaderProgram, "radius"), circle.Radius);
	glUniform1i(glGetUniformLocation(s_RendererData->CircleShaderProgram, "fill"), false);
	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
}

void Renderer::DrawBox(const LP::Box& box, const LP::Transform& tran, const glm::vec3& color)
{
	uint32_t offset = s_RendererData->LineVertices.size();
	glm::vec2 c = { box.Center.x + tran.P.x, box.Center.y + tran.P.y };
	glm::mat2 r{ tran.R.Cos, tran.R.Sin, -tran.R.Sin, tran.R.Cos };
	s_RendererData->LineVertices.emplace_back(RendererData::Vertex{ c + r * glm::vec2{  box.Size.x,  box.Size.y }, color });
	s_RendererData->LineVertices.emplace_back(RendererData::Vertex{ c + r * glm::vec2{  box.Size.x, -box.Size.y }, color });
	s_RendererData->LineVertices.emplace_back(RendererData::Vertex{ c + r * glm::vec2{ -box.Size.x, -box.Size.y }, color });
	s_RendererData->LineVertices.emplace_back(RendererData::Vertex{ c + r * glm::vec2{ -box.Size.x,  box.Size.y }, color });
	s_RendererData->LineIndices.push_back(offset + 0);
	s_RendererData->LineIndices.push_back(offset + 1);
	s_RendererData->LineIndices.push_back(offset + 1);
	s_RendererData->LineIndices.push_back(offset + 2);
	s_RendererData->LineIndices.push_back(offset + 2);
	s_RendererData->LineIndices.push_back(offset + 3);
	s_RendererData->LineIndices.push_back(offset + 3);
	s_RendererData->LineIndices.push_back(offset + 0);
}

void Renderer::DrawAABB(const LP::AABB& aabb, const glm::vec3& color)
{
	uint32_t offset = s_RendererData->TriangleVertices.size();
	s_RendererData->TriangleVertices.emplace_back(RendererData::Vertex{ { aabb.Max.x, aabb.Max.y }, color });
	s_RendererData->TriangleVertices.emplace_back(RendererData::Vertex{ { aabb.Max.x, aabb.Min.y }, color });
	s_RendererData->TriangleVertices.emplace_back(RendererData::Vertex{ { aabb.Min.x, aabb.Min.y }, color });
	s_RendererData->TriangleVertices.emplace_back(RendererData::Vertex{ { aabb.Min.x, aabb.Max.y }, color });
	s_RendererData->TriangleIndices.push_back(offset + 0);
	s_RendererData->TriangleIndices.push_back(offset + 1);
	s_RendererData->TriangleIndices.push_back(offset + 2);
	s_RendererData->TriangleIndices.push_back(offset + 2);
	s_RendererData->TriangleIndices.push_back(offset + 3);
	s_RendererData->TriangleIndices.push_back(offset + 0);
}

void Renderer::DrawLine(const LP::Vec2& start, const LP::Vec2& length, const glm::vec3& color)
{
	uint32_t offset = s_RendererData->LineVertices.size();
	s_RendererData->LineVertices.emplace_back(RendererData::Vertex{ ToGLM(start), color });
	s_RendererData->LineVertices.emplace_back(RendererData::Vertex{ ToGLM(start + length), color });
	s_RendererData->LineIndices.push_back(offset);
	s_RendererData->LineIndices.push_back(offset + 1);
}

void Renderer::DrawPoly(const LP::Polygon& poly, const LP::Transform& tran, const glm::vec3& color)
{
	uint32_t offset = s_RendererData->LineVertices.size();
	uint32_t size = poly.Count;
	
	glm::mat2 r{ tran.R.Cos, tran.R.Sin, -tran.R.Sin, tran.R.Cos };
	for (uint32_t i = 0; i < size; i++)
	{
		s_RendererData->LineVertices.emplace_back(RendererData::Vertex{ ToGLM(tran.P) + r * ToGLM(poly.Points[i]), color });
		s_RendererData->LineIndices.push_back(offset + i);
		s_RendererData->LineIndices.push_back(offset + (i + 1) % (size));
	}
}

void Renderer::End()
{
	// Render Lines
	glUseProgram(s_RendererData->LineShaderProgram);
	glBindVertexArray(s_RendererData->LineVertexArray);
	glBindBuffer(GL_ARRAY_BUFFER, s_RendererData->LineVertexBuffer);
	glBufferSubData(GL_ARRAY_BUFFER, 0, s_RendererData->LineVertices.size() * sizeof(RendererData::Vertex), s_RendererData->LineVertices.data());
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, s_RendererData->LineIndexBuffer);
	glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, s_RendererData->LineIndices.size() * sizeof(uint32_t), s_RendererData->LineIndices.data());
	glDrawElements(GL_LINES, s_RendererData->LineIndices.size(), GL_UNSIGNED_INT, 0);

	// Render Triangles
	glUseProgram(s_RendererData->TriangleShaderProgram);
	glBindVertexArray(s_RendererData->TriangleVertexArray);
	glBindBuffer(GL_ARRAY_BUFFER, s_RendererData->TriangleVertexBuffer);
	glBufferSubData(GL_ARRAY_BUFFER, 0, s_RendererData->TriangleVertices.size() * sizeof(RendererData::Vertex), s_RendererData->TriangleVertices.data());
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, s_RendererData->TriangleIndexBuffer);
	glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, s_RendererData->TriangleIndices.size() * sizeof(uint32_t), s_RendererData->TriangleIndices.data());
	glDrawElements(GL_TRIANGLES, s_RendererData->TriangleIndices.size(), GL_UNSIGNED_INT, 0);

}

void Renderer::EnableBlend()
{
	glEnable(GL_BLEND);
}

void Renderer::DisableBlend()
{
	glDisable(GL_BLEND);
}



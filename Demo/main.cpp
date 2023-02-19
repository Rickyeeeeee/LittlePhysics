#include <iostream>
#include <list>
#include <random>
#include <functional>
#include <queue>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "imgui/imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <LittlePhysics/LittlePhysics.h>
#include <LittlePhysics/CollisionNarrowPhase.h>
#include <LittlePhysics/World.h>
#include "renderer.h"
#include "utils.h"

static inline glm::vec2 ToGLM(LP::Vec2 vec)
{
	return { vec.x, vec.y };
}

bool isFocus = true;
int but = 1;

using namespace LP;
int  drawDbvhTreeLevel = 0;
bool deleteBody = false;
Renderer::Camera camera;

World *world;


int input[2] = { 0,0 };

std::ostream& operator<<(std::ostream& os, const LP::Vec2& vec)
{
	return os << vec.x << ", " << vec.y;
}

int main()
{
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	float width = 1000.0f;
	float height = 1000.0f;
	GLFWwindow* window = glfwCreateWindow(width, height, "DEMO", NULL, NULL);
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return -1;
	}
	glfwMakeContextCurrent(window);

	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
	{
		std::cout << "Failed to initialize GLAD" << std::endl;
		return -1;
	}
	glfwSetScrollCallback(window, [](GLFWwindow* window, double xOffset, double yOffset) {
		camera.Zoom += yOffset * 0.0005f + camera.Zoom * 0.04f * yOffset;
		});
	glfwSetWindowSizeCallback(window, [](GLFWwindow* window, int width, int height) {
		glViewport(0, 0, width, height);
		});
	glfwSetMouseButtonCallback(window, [](GLFWwindow* window, int button, int action, int mod) {
		if (button == GLFW_MOUSE_BUTTON_1 && action == GLFW_PRESS)
			isFocus = true;
		else
		{
			isFocus = false;
		}
		});
	glfwSetKeyCallback(window, [](GLFWwindow* window, int key, int scancode, int action, int mod) {
		if (action == GLFW_PRESS)
		{
			if (key == GLFW_KEY_LEFT)
				drawDbvhTreeLevel--;
			if (key == GLFW_KEY_RIGHT)
				drawDbvhTreeLevel++;
			if (key == GLFW_KEY_BACKSPACE)
				deleteBody = true;
		}
		});

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGui::StyleColorsDark();

	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 450");
	glfwSwapInterval(1);

	camera.Position = { 0.0f, 0.0f };
	camera.Zoom = 0.01f;
	world = new World();
	std::vector <Body*> Bodies;
	for (int i = 0; i < 3; i++)
	{
		BodyCreateInfo info;
		info.BodyType = BODY_TYPE::STATIC;
		info.Density = 0.10f;
		info.Restitution = 0.0f;
		info.Friction = 0.1f;
		auto body = world->CreateBody(&info);
		Bodies.push_back(body);
	}        


	Bodies[0]->AttachBoxShape({ 100.0f, 10.0f });
	Bodies[0]->SetType(BODY_TYPE::STATIC);
	Bodies[0]->SetPosition({ 0.0f, -200.0f });
	////Bodies[0]->SetRotation(glm::radians(20.0f));
	Bodies[1]->AttachBoxShape({ 11.0f, 200.0f });
	Bodies[1]->SetType(BODY_TYPE::STATIC);
	Bodies[1]->SetPosition({ 101.0f, 0.0f });
	Bodies[2]->AttachBoxShape({ 11.0f, 200.0f });
	Bodies[2]->SetType(BODY_TYPE::STATIC);
	Bodies[2]->SetPosition({ -101.0f, 0.0f });
	//std::vector<Vec2> vertices;
	//vertices.push_back({ 0.0f, 1.0f });
	//vertices.push_back({ -1.0f, 0.0f });
	//vertices.push_back({ -1.0f, -1.0f });
	//vertices.push_back({ 1.0f, -1.0f });
	//vertices.push_back({ 1.0f, 0.0f });
	//Bodies[4]->AttachPolygonShape(vertices.data(), vertices.size());
	//Bodies[3]->AttachBoxShape({ 4.0f, 4.0f });
	//Bodies[4]->AttachCircleShape(1.0f);

	//Bodies[5]->AttachBoxShape({ 1.0f, 3.0f });
	//Bodies[5]->SetType(BODY_TYPE::STATIC);
	//Bodies[5]->SetPosition({ 6.0f, 5.0f });

	LP::Transform tran;
	Renderer::Init();
	int space = 0;
	float degree = 0;
	float lastTime = glfwGetTime();

	std::random_device dev;
	std::mt19937 rng(dev());
	std::uniform_real_distribution<> dist(1.0f, 8.0f);
	double lastPos[2];

	// Imgui control variables
	bool showContactPoints = true;
	bool showContactNormals = false;
	bool showLocalPoints = false;
	bool simulating = true;
	while (!glfwWindowShouldClose(window))
	{
		float currentTime = glfwGetTime();
		float dt = (currentTime - lastTime);
		input[0] = 0;
		input[1] = 0;
		input[1] += (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS);
		input[1] -= (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS);
		input[0] += (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS);
		input[0] -= (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS);
		double pos[2];
		glfwGetCursorPos(window, &pos[0], &pos[1]);
		

			 tran.P.x = pos[0];
			 tran.P.y = pos[1];
			 tran.P -= LP::Vec2{ width, height } / 2.0f;
			 tran.P /= Vec2{ width, height } / 2.0f;
			 tran.P *= { 1.0f, -1.0f };
			 tran.P /= camera.Zoom;
			 tran.P.x -= camera.Position.x;
			 tran.P.y -= camera.Position.y;
		float deltaPos[2];
		deltaPos[0] = pos[0] - lastPos[0];
		deltaPos[1] = pos[1] - lastPos[1];
		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_2) == GLFW_PRESS)
		{
			float speed = 0.12f / camera.Zoom;
			camera.Position.x += deltaPos[0] * dt * speed;
			camera.Position.y -= deltaPos[1] * dt * speed;
		}
		static int lastspace = space;
		if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
		{
			space++;
		}
		else if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_RELEASE)
		{
			space = 0;
		}

		if (space > 0)
		{
			BodyCreateInfo info;
			info.BodyType = BODY_TYPE::DYNAMIC;
			info.Friction = 0.1f;
			info.FixRotation = false;
			info.Restitution = 0.0f;
			info.Density = 0.1f;
			auto* body = world->CreateBody(&info);
			body->SetPosition(tran.P);
			//body->SetRotation(glm::radians(30.0f));
			if (space % 3 == 1)
			{
				std::vector<Vec2> vertices;
				float length = (float)dist(rng);
				vertices.push_back({ 0.0f, length });
				vertices.push_back({ -length, -length });
				vertices.push_back({ length, -length });
				body->AttachPolygonShape(vertices.data(), vertices.size());
				//body->AttachCircleShape(3.0f);
			}
			else if (space % 3 == 2)
				body->AttachBoxShape({(float)dist(rng), (float)dist(rng) });
			//body->AttachBoxShape({ 4.0F, 4.0F });
			else
			{

				body->AttachCircleShape(dist(rng));
				//body->SetPosition(tran.P + Vec2{ 0.0f, 20.0f });
			}
				//body->AttachCircleShape(3.0f);
				//body->AttachBoxShape({ 3.0F, 3.0F });
			Bodies.push_back(body);
		}
		if (degree >= 360.0f)
			degree = 0.0f;
		//tran.R.Set(glm::radians(degree));
		//tran.R.Set(glm::radians(45.0f
		//Bodies[5]->ApplyForce(Vec2{ float(input[0]), float(input[1]) } * 500.0f);
		if (deleteBody && !Bodies.empty())
		{
			world->DeleteBody(Bodies.back());
			Bodies.pop_back();
			deleteBody = false;	
		}
		if (simulating)
			world->Step(0.01);

		//if (Bodies.size() > 1)
			 //Bodies[3]->SetPosition(tran.P);
		// Bodies[4]->SetRotation(40.0f);


		const auto& contacts = world->GetContacts();
		const auto& dbvhTree = world->GetDbvhTree();
		
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(0.1f, 0.1f, 0.1f, 1.0);
		Renderer::Begin(camera);

		//Renderer::DisableBlend();
		glm::vec3 aabbColor[6] = {
			{ 1.0f, 0.0f, 0.0f },
			{ 0.0f, 1.0f, 0.0f },
			{ 0.0f, 0.0f, 1.0f },
			{ 1.0f, 1.0f, 0.0f },
			{ 0.0f, 1.0f, 1.0f },
			{ 1.0f, 0.0f, 1.0f },
		};
		std::queue<LP::DbvhTree::Index> nodes;
		auto root = dbvhTree.m_Root;
		std::function<void(LP::DbvhTree::Index index, int level)> DrawDebugAABBRecur;
		DrawDebugAABBRecur = [&dbvhTree, &DrawDebugAABBRecur, &aabbColor](LP::DbvhTree::Index index, int level) {
			if (level == 0 || index == -1) return;
			level--;
			auto& node = dbvhTree.m_Nodes[index];
			auto aabb = node.AaBb;
			auto cx = (aabb.Max + aabb.Min) / 2.0f;
			aabb.Max = (aabb.Max - cx) * 1.2f + cx;
			aabb.Min = (aabb.Min - cx) * 1.2f + cx;
			Renderer::DrawAABB(node.AaBb, aabbColor[(drawDbvhTreeLevel - level) % 6]);
			DrawDebugAABBRecur(node.Child[0], level);
			DrawDebugAABBRecur(node.Child[1], level);
		};
		DrawDebugAABBRecur(root, drawDbvhTreeLevel);

		//Renderer::DisableBlend();
		for (Body* body : Bodies)
		{
			glm::vec3 color = { 1.0f, 1.0f, 1.0f };
			Shape* shape;
			auto type = body->GetShape(shape);
			Transform tr = body->GetTransform();
			switch (type)
			{
			case LP::COLLISION_SHAPE_TYPE::CIRCLE:
				Renderer::DrawCircle((LP::Circle&)*shape, tr, color);
				break;
			case LP::COLLISION_SHAPE_TYPE::BOX:
				Renderer::DrawBox((LP::Box&)*shape, tr, color);
				break;	
			case LP::COLLISION_SHAPE_TYPE::POLYGON:
				Renderer::DrawPoly((LP::Polygon&)*shape, tr, color);
				break;	
			default:
				break;
			}
		}

		for (const auto& contact : contacts)
		{

		/*	std::cout << "Count: " << contact.info.Count << "\n";
			std::cout << "Depths 1: " << contact.info.Depths[0] << "\n";
			std::cout << "Ref Point[0]: " << contact.info.RefPoints[0] << "\n";
			std::cout << "Ref Point[1]: " << contact.info.RefPoints[1] << "\n";*/

			if (showContactNormals)
			{
				Renderer::DrawLine(contact.info.Points[0], contact.info.Normal * contact.info.Depths[0] * 10.0f, { 1.0f, 0.0f, 0.0f });
				Renderer::DrawLine(contact.info.Points[0], contact.info.Normal * 2.0f, { 1.0f, 0.0f, 0.0f });
			}
			if (showContactPoints)
			{
				Renderer::DrawCircle(ToGLM(contact.info.Points[0]), 0.6f, { 1.0f, 0.0f, 0.0f });
				Renderer::DrawCircle(ToGLM(contact.info.Points[0]), 0.6f, { 1.0f, 0.0f, 0.0f });
			}
			if (showLocalPoints)
			{
				if (contact.info.Type == CONTACT_TYPE::EDGE_A)
				{
					Renderer::DrawCircle(ToGLM(contact.BodyA->GetTransform() * contact.info.RefPoints[1]), 0.6f, { 1.0f, 0.0f, 1.0f });
					Renderer::DrawCircle(ToGLM(contact.BodyA->GetTransform() * contact.info.RefPoints[0]), 0.6f, { 0.0f, 0.0f, 1.0f });
				}
				else
				{
					Renderer::DrawCircle(ToGLM(contact.BodyB->GetTransform() * contact.info.RefPoints[1]), 0.6f, { 1.0f, 0.0f, 1.0f });
					Renderer::DrawCircle(ToGLM(contact.BodyB->GetTransform() * contact.info.RefPoints[0]), 0.6f, { 0.0f, 0.0f, 1.0f });
				}
			}
			if (contact.info.Count > 1)
			{
				if (showContactNormals)
				{
					Renderer::DrawLine(contact.info.Points[1], contact.info.Normal * contact.info.Depths[0] * 10.0f, { 1.0f, 0.0f, 0.0f });
					Renderer::DrawLine(contact.info.Points[1], contact.info.Normal * 2.0f, { 1.0f, 0.0f, 0.0f });
				}
				if (showContactPoints)
				{
					Renderer::DrawCircle(ToGLM(contact.info.Points[1]), 0.6f, { 1.0f, 0.0f, 0.0f });
				}
			}
		}

		Renderer::End();


		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();
			ImGui::Begin("Debug");
			ImGui::Text("Press space bar to randomly create objects from mouse position");
			ImGui::Checkbox("Show Contact Points", &showContactPoints);
			ImGui::Checkbox("Show Contact Normals", &showContactNormals);
			ImGui::Checkbox("Show Local Points", &showLocalPoints);
			ImGui::SliderInt("Draw Debug dbvhTree Level", &drawDbvhTreeLevel, -1, 5);
			ImGui::Text("Total Body Count: %d.", world->GetBodyCount());
			ImGui::Text("Total Collision Pair Count: %d", dbvhTree.GetCollisionPairsCount());
			ImGui::Text("Total Contact Count: %d", world->GetContactCount());
			ImGui::Text("Time: %.3f ms", dt * 1000.0f);
			ImGui::Checkbox("Sleep", &world->GetSleep());
			if (ImGui::Button("Pause"))
				simulating = simulating ? false : true;
			if (ImGui::Button("Restart"))
			{
				uint32 count = Bodies.size() - 3;
				for (uint32 i = 0; i < count; i++)
				{
					auto* body = Bodies.back();
					world->DeleteBody(body);
					Bodies.pop_back();
				}
			}
			ImGui::End();
		ImGui::EndFrame();
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		glfwSwapBuffers(window);
		glfwPollEvents();
		lastTime = currentTime;
		lastPos[0] = pos[0];
		lastPos[1] = pos[1];
	}

	glfwTerminate();
	return 0;
}



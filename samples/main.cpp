/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty.
*/

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <getopt.h>
#include <string.h>
#include <limits.h>
#include <chrono>
#include <stdint.h>
#include <locale.h>

#ifndef HEADLESS
#include "imgui/imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"

#define GLFW_INCLUDE_NONE
#include "glad/glad.h"
#include "GLFW/glfw3.h"
#endif

#include "box2d-lite/World.h"
#include "box2d-lite/Body.h"
#include "box2d-lite/Joint.h"

struct PrgArgs
{
	int demo;
	bool headless, stepLimit, log;
	int64_t steps;
	size_t numBodies;
	const char* logFile;
};

#define MIN_NUM_BODIES 200
#define MIN_NUM_JOINTS 100
#define NUM_BODIES_MARGIN 10

#define MAX(ln, rn) ((ln) > (rn) ? (ln) : (rn))

namespace
{
#ifndef HEADLESS
	GLFWwindow* mainWindow = NULL;
#endif

	Body *bodies;
	Joint *joints;
	
	Body* bomb = NULL;

	float timeStep = 1.0f / 60.0f;
	int iterations = 10;
	Vec2 gravity(0.0f, -10.0f);
	Vec2 noGravity(0.0f, 0.0f);

	int maxBodies, maxJoints;

	PrgArgs args = {0, false, false, false, 0, MIN_NUM_BODIES - NUM_BODIES_MARGIN, NULL};

	int numBodies = 0;
	int numJoints = 0;

	int demoIndex = 0;

	int width = 1280;
	int height = 720;
	float zoom = 10.0f;
	float pan_y = 8.0f;
	float pan_x = 0.0f;
	bool dragging = false;
	double lastMouseX = 0.0, lastMouseY = 0.0;

	World world(gravity, iterations);
}

#ifndef HEADLESS
static void glfwErrorCallback(int error, const char* description)
{
	printf("GLFW error %d: %s\n", error, description);
}

static void DrawText(int x, int y, const char* string)
{
	ImVec2 p;
	p.x = float(x);
	p.y = float(y);
	ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
	ImGui::SetCursorPos(p);
	ImGui::TextColored(ImColor(230, 153, 153, 255), "%s", string);
	ImGui::End();
}

static void DrawBody(Body* body)
{
	Mat22 R(body->rotation);
	Vec2 x = body->position;
	Vec2 h = 0.5f * body->width;

	Vec2 v1 = x + R * Vec2(-h.x, -h.y);
	Vec2 v2 = x + R * Vec2( h.x, -h.y);
	Vec2 v3 = x + R * Vec2( h.x,  h.y);
	Vec2 v4 = x + R * Vec2(-h.x,  h.y);

	if (body == bomb)
		glColor3f(0.4f, 0.9f, 0.4f);
	else
		glColor3f(0.8f, 0.8f, 0.9f);

	glBegin(GL_LINE_LOOP);
	glVertex2f(v1.x, v1.y);
	glVertex2f(v2.x, v2.y);
	glVertex2f(v3.x, v3.y);
	glVertex2f(v4.x, v4.y);
	glEnd();
}

static void DrawJoint(Joint* joint)
{
	Body* b1 = joint->body1;
	Body* b2 = joint->body2;

	Mat22 R1(b1->rotation);
	Mat22 R2(b2->rotation);

	Vec2 x1 = b1->position;
	Vec2 p1 = x1 + R1 * joint->localAnchor1;

	Vec2 x2 = b2->position;
	Vec2 p2 = x2 + R2 * joint->localAnchor2;

	glColor3f(0.5f, 0.5f, 0.8f);
	glBegin(GL_LINES);
	glVertex2f(x1.x, x1.y);
	glVertex2f(p1.x, p1.y);
	glVertex2f(x2.x, x2.y);
	glVertex2f(p2.x, p2.y);
	glEnd();
}
#endif

#define ASSERT_BOUNDS \
do { \
	assert(numBodies <= maxBodies); \
 	assert(numJoints <= maxJoints); \
} while (false)

static void LaunchBomb()
{
	if (!bomb)
	{
		bomb = bodies + numBodies;
		bomb->Set(Vec2(1.0f, 1.0f), 50.0f);
		bomb->friction = 0.2f;
		world.Add(bomb);
		numBodies++; ASSERT_BOUNDS;
	}

	bomb->position.Set(Random(-15.0f, 15.0f), 15.0f);
	bomb->rotation = Random(-1.5f, 1.5f);
	bomb->velocity = -1.5f * bomb->position;
	bomb->angularVelocity = Random(-20.0f, 20.0f);
}

// Single box
static void Demo1(Body* b, Joint* j)
{
	b->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b->position.Set(0.0f, -0.5f * b->width.y);
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	b->Set(Vec2(1.0f, 1.0f), 200.0f);
	b->position.Set(0.0f, 4.0f);
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;
}

// A simple pendulum
static void Demo2(Body* b, Joint* j)
{
	Body* b1 = b + 0;
	b1->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b1->friction = 0.2f;
	b1->position.Set(0.0f, -0.5f * b1->width.y);
	b1->rotation = 0.0f;
	world.Add(b1);

	Body* b2 = b + 1;
	b2->Set(Vec2(1.0f, 1.0f), 100.0f);
	b2->friction = 0.2f;
	b2->position.Set(9.0f, 11.0f);
	b2->rotation = 0.0f;
	world.Add(b2);

	numBodies += 2; ASSERT_BOUNDS;

	j->Set(b1, b2, Vec2(0.0f, 11.0f));
	world.Add(j);

	numJoints += 1; ASSERT_BOUNDS;
}

// Varying friction coefficients
static void Demo3(Body* b, Joint* j)
{
	b->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b->position.Set(0.0f, -0.5f * b->width.y);
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	b->Set(Vec2(13.0f, 0.25f), FLT_MAX);
	b->position.Set(-2.0f, 11.0f);
	b->rotation = -0.25f;
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	b->Set(Vec2(0.25f, 1.0f), FLT_MAX);
	b->position.Set(5.25f, 9.5f);
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	b->Set(Vec2(13.0f, 0.25f), FLT_MAX);
	b->position.Set(2.0f, 7.0f);
	b->rotation = 0.25f;
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	b->Set(Vec2(0.25f, 1.0f), FLT_MAX);
	b->position.Set(-5.25f, 5.5f);
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	b->Set(Vec2(13.0f, 0.25f), FLT_MAX);
	b->position.Set(-2.0f, 3.0f);
	b->rotation = -0.25f;
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	float friction[5] = {0.75f, 0.5f, 0.35f, 0.1f, 0.0f};
	for (int i = 0; i < 5; ++i)
	{
		b->Set(Vec2(0.5f, 0.5f), 25.0f);
		b->friction = friction[i];
		b->position.Set(-7.5f + 2.0f * i, 14.0f);
		world.Add(b);
		++b; numBodies++; ASSERT_BOUNDS;
	}
}

// A vertical stack
static void Demo4(Body* b, Joint* j)
{
	b->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.0f, -0.5f * b->width.y);
	b->rotation = 0.0f;
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	for (int i = 0; i < 10; ++i)
	{
		b->Set(Vec2(1.0f, 1.0f), 1.0f);
		b->friction = 0.2f;
		float x = Random(-0.1f, 0.1f);
		b->position.Set(x, 0.51f + 1.05f * i);
		world.Add(b);
		++b; numBodies++; ASSERT_BOUNDS;
	}
}

// A pyramid
static void Demo5(Body* b, Joint* j)
{
	b->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.0f, -0.5f * b->width.y);
	b->rotation = 0.0f;
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	Vec2 x(-6.0f, 0.75f);
	Vec2 y;

	for (int i = 0; i < 12; ++i)
	{
		y = x;

		for (int j = i; j < 12; ++j)
		{
			b->Set(Vec2(1.0f, 1.0f), 10.0f);
			b->friction = 0.2f;
			b->position = y;
			world.Add(b);
			++b; numBodies++; ASSERT_BOUNDS;

			y += Vec2(1.125f, 0.0f);
		}

		//x += Vec2(0.5625f, 1.125f);
		x += Vec2(0.5625f, 2.0f);
	}
}

// A teeter
static void Demo6(Body* b, Joint* j)
{
	Body* b1 = b + 0;
	b1->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b1->position.Set(0.0f, -0.5f * b1->width.y);
	world.Add(b1);

	Body* b2 = b + 1;
	b2->Set(Vec2(12.0f, 0.25f), 100.0f);
	b2->position.Set(0.0f, 1.0f);
	world.Add(b2);

	Body* b3 = b + 2;
	b3->Set(Vec2(0.5f, 0.5f), 25.0f);
	b3->position.Set(-5.0f, 2.0f);
	world.Add(b3);

	Body* b4 = b + 3;
	b4->Set(Vec2(0.5f, 0.5f), 25.0f);
	b4->position.Set(-5.5f, 2.0f);
	world.Add(b4);

	Body* b5 = b + 4;
	b5->Set(Vec2(1.0f, 1.0f), 100.0f);
	b5->position.Set(5.5f, 15.0f);
	world.Add(b5);

	numBodies += 5; ASSERT_BOUNDS;

	j->Set(b1, b2, Vec2(0.0f, 1.0f));
	world.Add(j);

	numJoints += 1; ASSERT_BOUNDS;
}

// A suspension bridge
static void Demo7(Body* b, Joint* j)
{
	b->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.0f, -0.5f * b->width.y);
	b->rotation = 0.0f;
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	const int numPlanks = 15;
	float mass = 50.0f;

	for (int i = 0; i < numPlanks; ++i)
	{
		b->Set(Vec2(1.0f, 0.25f), mass);
		b->friction = 0.2f;
		b->position.Set(-8.5f + 1.25f * i, 5.0f);
		world.Add(b);
		++b; numBodies++; ASSERT_BOUNDS;
	}

	// Tuning
	float frequencyHz = 2.0f;
	float dampingRatio = 0.7f;

	// frequency in radians
	float omega = 2.0f * k_pi * frequencyHz;

	// damping coefficient
	float d = 2.0f * mass * dampingRatio * omega;

	// spring stifness
	float k = mass * omega * omega;

	// magic formulas
	float softness = 1.0f / (d + timeStep * k);
	float biasFactor = timeStep * k / (d + timeStep * k);

	for (int i = 0; i < numPlanks; ++i)
	{
		j->Set(bodies+i, bodies+i+1, Vec2(-9.125f + 1.25f * i, 5.0f));
		j->softness = softness;
		j->biasFactor = biasFactor;

		world.Add(j);
		++j; numJoints++; ASSERT_BOUNDS;
	}

	j->Set(bodies + numPlanks, bodies, Vec2(-9.125f + 1.25f * numPlanks, 5.0f));
	j->softness = softness;
	j->biasFactor = biasFactor;
	world.Add(j);
	++j; numJoints++; ASSERT_BOUNDS;
}

// Dominos
static void Demo8(Body* b, Joint* j)
{
	Body* b1 = b;
	b->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b->position.Set(0.0f, -0.5f * b->width.y);
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	b->Set(Vec2(12.0f, 0.5f), FLT_MAX);
	b->position.Set(-1.5f, 10.0f);
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	for (int i = 0; i < 10; ++i)
	{
		b->Set(Vec2(0.2f, 2.0f), 10.0f);
		b->position.Set(-6.0f + 1.0f * i, 11.125f);
		b->friction = 0.1f;
		world.Add(b);
		++b; numBodies++; ASSERT_BOUNDS;
	}

	b->Set(Vec2(14.0f, 0.5f), FLT_MAX);
	b->position.Set(1.0f, 6.0f);
	b->rotation = 0.3f;
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	Body* b2 = b;
	b->Set(Vec2(0.5f, 3.0f), FLT_MAX);
	b->position.Set(-7.0f, 4.0f);
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	Body* b3 = b;
	b->Set(Vec2(12.0f, 0.25f), 20.0f);
	b->position.Set(-0.9f, 1.0f);
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	j->Set(b1, b3, Vec2(-2.0f, 1.0f));
	world.Add(j);
	++j; numJoints++; ASSERT_BOUNDS;

	Body* b4 = b;
	b->Set(Vec2(0.5f, 0.5f), 10.0f);
	b->position.Set(-10.0f, 15.0f);
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	j->Set(b2, b4, Vec2(-7.0f, 15.0f));
	world.Add(j);
	++j; numJoints++; ASSERT_BOUNDS;

	Body* b5 = b;
	b->Set(Vec2(2.0f, 2.0f), 20.0f);
	b->position.Set(6.0f, 2.5f);
	b->friction = 0.1f;
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	j->Set(b1, b5, Vec2(6.0f, 2.6f));
	world.Add(j);
	++j; numJoints++; ASSERT_BOUNDS;

	Body* b6 = b;
	b->Set(Vec2(2.0f, 0.2f), 10.0f);
	b->position.Set(6.0f, 3.6f);
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	j->Set(b5, b6, Vec2(7.0f, 3.5f));
	world.Add(j);
	++j; numJoints++; ASSERT_BOUNDS;
}

// A multi-pendulum
static void Demo9(Body* b, Joint* j)
{
	b->Set(Vec2(100.0f, 20.0f), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.0f, -0.5f * b->width.y);
	b->rotation = 0.0f;
	world.Add(b);

	Body * b1 = b;
	++b;
	numBodies++; ASSERT_BOUNDS;

	float mass = 10.0f;

	// Tuning
	float frequencyHz = 4.0f;
	float dampingRatio = 0.7f;

	// frequency in radians
	float omega = 2.0f * k_pi * frequencyHz;

	// damping coefficient
	float d = 2.0f * mass * dampingRatio * omega;

	// spring stiffness
	float k = mass * omega * omega;

	// magic formulas
	float softness = 1.0f / (d + timeStep * k);
	float biasFactor = timeStep * k / (d + timeStep * k);

	const float y = 12.0f;

	for (int i = 0; i < 15; ++i)
	{
		Vec2 x(0.5f + i, y);
		b->Set(Vec2(0.75f, 0.25f), mass);
		b->friction = 0.2f;
		b->position = x;
		b->rotation = 0.0f;
		world.Add(b);

		j->Set(b1, b, Vec2(float(i), y));
		j->softness = softness;
		j->biasFactor = biasFactor;
		world.Add(j);

		b1 = b;
		++b;
		numBodies++; ASSERT_BOUNDS;
		++j;
		numJoints++; ASSERT_BOUNDS;
	}
}

static void Demo10(Body* b, Joint* j)
{
	const int size = sqrt(args.numBodies);
	const int nrows = size, ncols = size;
	const float yoffset = 10.0f;

	b->Set(Vec2(ncols * 8.0f, 20.0f), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.0f, -0.5f * b->width.y);
	b->rotation = 0.0f;
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	for (int i = 0; i < nrows; ++i)
	{
		for (int j = -ncols/2; j < (ncols+1)/2; ++j)
		{
			b->Set(Vec2(1.0f, 1.0f), 1.0f);
			b->friction = 0.2f;
			b->position.Set(0.51f + 1.05f * j, yoffset + 0.51f + 1.05f * i);
			world.Add(b);
			++b; numBodies++; ASSERT_BOUNDS;
		}
	}
}

static void Demo11(Body* b, Joint* j)
{
	const int size = sqrt(args.numBodies);

	const float bodyOffset = 10.0f;
	const float wallLength = size * bodyOffset;

	b->Set(Vec2(wallLength, 1.0f), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.0f, -0.5f - wallLength * 0.5f);
	b->rotation = 0.0f;
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	b->Set(Vec2(wallLength, 1.0f), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.0f, 0.5f + wallLength * 0.5f);
	b->rotation = 0.0f;
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	b->Set(Vec2(1.0f, wallLength), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(-0.5f - wallLength * 0.5f, 0.0f);
	b->rotation = 0.0f;
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	b->Set(Vec2(1.0f, wallLength), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.5f + wallLength * 0.5f, 0.0f);
	b->rotation = 0.0f;
	world.Add(b);
	++b; numBodies++; ASSERT_BOUNDS;

	const int maxForce = 100.0f;

	for (int i = 0; i < args.numBodies; ++i)
	{
		b->Set(Vec2(1.0f, 1.0f), 1.0f);
		b->friction = 0.0f;
		float x = Random(-0.5 - wallLength / 2, 0.5 + wallLength / 2);
		float y = Random(-0.5 - wallLength / 2, 0.5 + wallLength / 2);
		b->position.Set(x, y);
		b->force.Set(Random(-maxForce, maxForce), Random(-maxForce, maxForce));
		world.Add(b);
		++b; numBodies++; ASSERT_BOUNDS;
	}

	world.gravity = noGravity;
}

#define NUM_BODIES_ROPE 30
static void Demo12(Body* b, Joint* j)
{
	int numRopes = args.numBodies / NUM_BODIES_ROPE;
	numRopes = numRopes == 0 ? 1 : numRopes;
	const int&& numJoinsRope = NUM_BODIES_ROPE;

	b->Set(Vec2(1.0f, 1.0f), FLT_MAX);
	b->friction = 0.2f;
	b->position.Set(0.0f, 2.0f);
	b->rotation = 0.0f;
	world.Add(b);

	Body * bFloor = b;
	++b;
	numBodies++; ASSERT_BOUNDS;

	float mass = 10.0f;

	// Tuning
	float frequencyHz = 4.0f;
	float dampingRatio = 0.7f;

	// frequency in radians
	float omega = 2.0f * k_pi * frequencyHz;

	// damping coefficient
	float d = 2.0f * mass * dampingRatio * omega;

	// spring stiffness
	float k = mass * omega * omega;

	// magic formulas
	float softness = 1.0f / (d + timeStep * k);
	float biasFactor = timeStep * k / (d + timeStep * k);

	const float ropeXOffset = 1.0f, ropeYOffset = 0.5f;
	const float ropeInitY = 0.0f;

	for (int ir = 0; ir < numRopes; ++ir) {
		Body * b1 = bFloor;
		const float xOffset = (ir - (numRopes / 2)) * ropeXOffset;;
		const float y = ropeInitY - ropeYOffset * (numRopes - ir);

		for (int ib = 0; ib < NUM_BODIES_ROPE; ++ib)
		{
			Vec2 x(xOffset + 0.5f + ib, y);
			b->Set(Vec2(0.75f, 0.25f), mass);
			b->friction = 0.2f;
			b->position = x;
			b->rotation = 0.0f;
			world.Add(b);

			j->Set(b1, b, Vec2(xOffset + ib, y));
			j->softness = softness;
			j->biasFactor = biasFactor;
			world.Add(j);

			b1 = b;
			++b;
			numBodies++; ASSERT_BOUNDS;
			++j;
			numJoints++; ASSERT_BOUNDS;
		}
	}
}
static int getMaxUsedJoints() {
	int numRopes = args.numBodies / NUM_BODIES_ROPE;
	numRopes = numRopes == 0 ? 1 : numRopes;
	const int&& numJoinsRope = NUM_BODIES_ROPE;
	return MAX(numJoinsRope * numRopes, MIN_NUM_JOINTS);
}

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
void (*demos[])(Body* b, Joint* j) = {Demo1, Demo2, Demo3, Demo4, Demo5, Demo6, Demo7, Demo8, Demo9, Demo10, Demo11, Demo12};
const char* demoStrings[] = {
	"Demo 1: A Single Box",
	"Demo 2: Simple Pendulum",
	"Demo 3: Varying Friction Coefficients",
	"Demo 4: Randomized Stacking",
	"Demo 5: Pyramid Stacking",
	"Demo 6: A Teeter",
	"Demo 7: A Suspension Bridge",
	"Demo 8: Dominos",
	"Demo 9: Multi-pendulum",
	"Demo 10: Rubik",
	"Demo 11: Space",
	"Demo 12: Ropes"
};

static void InitDemo(int index)
{
	world.Clear();
	numBodies = 0;
	numJoints = 0;
	bomb = NULL;

	demoIndex = index;
	demos[index](bodies, joints);
}

#ifndef HEADLESS
static void Reshape(GLFWwindow*, int w, int h)
{
	width = w;
	height = h > 0 ? h : 1;

	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	float aspect = float(width) / float(height);
	if (width >= height)
	{
		// aspect >= 1, set the height from -1 to 1, with larger width
		glOrtho(-zoom * aspect + pan_x, zoom * aspect + pan_x,
			-zoom + pan_y, zoom + pan_y,
			-1.0, 1.0);
	}
	else
	{
		// aspect < 1, set the width to -1 to 1, with larger height
		glOrtho(-zoom + pan_x, zoom + pan_x,
			-zoom / aspect + pan_y, zoom / aspect + pan_y,
			-1.0, 1.0);
	}
}

static void Keyboard(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (action != GLFW_PRESS && action != GLFW_REPEAT)
	{
		return;
	}

	switch (key)
	{
	case GLFW_KEY_ESCAPE:
		// Quit
		glfwSetWindowShouldClose(mainWindow, GL_TRUE);
		break;

	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
	case '9':
		InitDemo(key - GLFW_KEY_1);
		break;

	case GLFW_KEY_F1:
	case GLFW_KEY_F2:
	case GLFW_KEY_F3:
		InitDemo(9 + key - GLFW_KEY_F1);
		break;

	case GLFW_KEY_A:
		World::accumulateImpulses = !World::accumulateImpulses;
		break;

	case GLFW_KEY_P:
		World::positionCorrection = !World::positionCorrection;
		break;

	case GLFW_KEY_W:
		World::warmStarting = !World::warmStarting;
		break;

	case GLFW_KEY_G:
		if (world.gravity.y == 0.0f)
			world.gravity = gravity;
		else
			world.gravity = noGravity;
		break;

	case GLFW_KEY_SPACE:
		LaunchBomb();
		break;

	case GLFW_KEY_LEFT:
		pan_x -= 1.0f;
		Reshape(window, width, height);
		break;

	case GLFW_KEY_RIGHT:
		pan_x += 1.0f;
		Reshape(window, width, height);
		break;

	case GLFW_KEY_UP:
		pan_y += 1.0f;
		Reshape(window, width, height);
		break;

	case GLFW_KEY_DOWN:
		pan_y -= 1.0f;
		Reshape(window, width, height);
		break;

	case GLFW_KEY_KP_ADD:
	case GLFW_KEY_EQUAL:
		zoom /= 1.1f;
		if (zoom < 0.1f)
			zoom = 0.1f;
		Reshape(window, width, height);
		break;

	case GLFW_KEY_KP_SUBTRACT:
	case GLFW_KEY_MINUS:
		zoom *= 1.1f;
		Reshape(window, width, height);
		break;
	}
}

static void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset)
{
    float zoomFactor = 1.1f;

    if (yoffset > 0)
        zoom /= zoomFactor; // zoom in
    else
        zoom *= zoomFactor; // zoom out

    if (zoom < 0.1f) zoom = 0.1f;

    Reshape(window, width, height);
}

static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_LEFT)
    {
        if (action == GLFW_PRESS)
        {
            dragging = true;
            glfwGetCursorPos(window, &lastMouseX, &lastMouseY);
        }
        else if (action == GLFW_RELEASE)
        {
            dragging = false;
        }
    }
}

static void CursorPosCallback(GLFWwindow* window, double xpos, double ypos)
{
    if (!dragging)
        return;

    double dx = xpos - lastMouseX;
    double dy = ypos - lastMouseY;

    lastMouseX = xpos;
    lastMouseY = ypos;

    float aspect = float(width) / float(height);

    float scale = zoom / height * 2.0f;

    pan_x -= dx * scale;
    pan_y += dy * scale;

    Reshape(window, width, height);
}
#endif

static const struct option longOpts[] = {
	{"help", 			no_argument, 		0, 'h'},
	{"demo", 			required_argument, 	0, 'd'},
	{"headless", 	no_argument, 			0, 'e'},
	{"steps", 		required_argument, 		0, 's'},
	{"bodies", 		required_argument, 		0, 'b'},
	{"output", 		optional_argument, 		0, 'o'},
	{0, 					0, 				0, 0  }
};
static const char* opts = (
	"h"
	"d:"
	"e"
	"s:"
	"b:"
	"o::"
);
static const char* optInfo[] = {
	"display this help",
	"demo number",
	"enable headless mode, -s or --step is required",
	"exit when the specified number of frames is exceeded",
	"number of bodies in the demo",
	"output file for step time logging or stdin if no argument specified"
};
static const char* argInfo[] = {
	0,
	"integer",
	0,
	"integer",
	"file name"
};

static void printInfo(const char* prgName)
{
	int optIdx = 0;
	const struct option* opt;
	printf("Usage: %s [OPTION]...\n", prgName);
	printf("Optimized and parallelized Box2D-lite physics engine\n");
	printf("\n");
	printf("Options:\n");
	while ((opt = &longOpts[optIdx])->name != 0)
	{
		bool hasArg = opt->has_arg == required_argument;
	char optBuffer[128];
	snprintf(
		optBuffer, sizeof(optBuffer),
		"-%c, --%s%s%s%s",
		opt->val,
		opt->name,
		hasArg ? "=[" : "",
		hasArg ? argInfo[optIdx] : "",
		hasArg ? "]" : ""
	);
	printf("  %-30s %s\n", optBuffer, optInfo[optIdx]);
		optIdx++;
	}
}

static int parseArgv(int argc, char* const* argv)
{
	const char* prgName = rindex(argv[0], '/') + 1;
	int opt;

  while ((opt = getopt_long(argc, argv, opts, longOpts, NULL)) != -1)
  {
	switch (opt)
	{
	case 'h':
		printInfo(prgName);
		return -1;
	case 'd':
		{
			int demo = atoi(optarg) - 1;
			if (demo < ARRAY_SIZE(demos))
				args.demo = demo;
			else
			{
				printf("There is no \"%d\" demo; available demos: 1-%ld\n",
					demo + 1, ARRAY_SIZE(demos));
				return -1;
			}
		}
		break;
	case 'e':
		args.headless = true;
		break;
	case 's':
		args.stepLimit = true;
		args.steps = strtoul(optarg, NULL, 10);
		break;
	case 'b':
		args.numBodies = strtoul(optarg, NULL, 10);
		break;
	case 'o':
		args.log = true;
		args.logFile = optarg;
		break;
	default:
		printInfo(prgName);
		return -1;
	}
  }

  if (
#ifndef HEADLESS
	args.headless &&
#endif
	!args.stepLimit
  )
  {
  	printInfo(prgName);
		return -1;
  }
  return 0;
}

static FILE* openLogFile()
{
	FILE* logFile = NULL;

	if (args.log)
	{
		logFile = args.logFile ? fopen(args.logFile, "w+") : stdout;
		if (!logFile)
		{
			perror("File opening failed");
			return NULL;
		}
	}
	return logFile;
}

static void writeLogStepNs(FILE* logFile, int64_t stepNs)
{
	if (args.log)
	{
		fprintf(logFile, "%ld\n", stepNs);
		fflush(logFile);
	}
}

static void closeLogFile(FILE* logFile)
{
	if (args.log && logFile != stdout)
	{
		fclose(logFile);
	}
}

#ifndef HEADLESS
static int runDemo()
{
	FILE* logFile = openLogFile();

	glfwSetErrorCallback(glfwErrorCallback);

	if (glfwInit() == 0)
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

	mainWindow = glfwCreateWindow(width, height, "box2d-lite", NULL, NULL);
	if (mainWindow == NULL)
	{
		fprintf(stderr, "Failed to open GLFW mainWindow.\n");
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(mainWindow);

	// Load OpenGL functions using glad
	int gladStatus = gladLoadGL();
	if (gladStatus == 0)
	{
		fprintf(stderr, "Failed to load OpenGL.\n");
		glfwTerminate();
		return -1;
	}

	glfwSwapInterval(1);
	glfwSetWindowSizeCallback(mainWindow, Reshape);
	glfwSetKeyCallback(mainWindow, Keyboard);
	glfwSetScrollCallback(mainWindow, ScrollCallback);
	glfwSetMouseButtonCallback(mainWindow, MouseButtonCallback);
	glfwSetCursorPosCallback(mainWindow, CursorPosCallback);

	float xscale, yscale;
	glfwGetWindowContentScale(mainWindow, &xscale, &yscale);
	float uiScale = xscale;

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGui::StyleColorsClassic();
	ImGui_ImplGlfw_InitForOpenGL(mainWindow, true);
	ImGui_ImplOpenGL2_Init();
	ImGuiIO& io = ImGui::GetIO();
	io.FontGlobalScale = uiScale;

	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	
	float aspect = float(width) / float(height);
	if (width >= height)
	{
		// aspect >= 1, set the height from -1 to 1, with larger width
		glOrtho(-zoom * aspect, zoom * aspect, -zoom + pan_y, zoom + pan_y, -1.0, 1.0);
	}
	else
	{
		// aspect < 1, set the width to -1 to 1, with larger height
		glOrtho(-zoom, zoom, -zoom / aspect + pan_y, zoom / aspect + pan_y, -1.0, 1.0);
	}

	const double infoRefreshSecs = 1.0;

	int64_t step = 0;

	int frameCounter = 1;
	double lastFrameTime;
	double frameSecsAccum = 0.0;
	int64_t stepNsAcum = 0;

	int avgFPS = 0;
	int64_t avgStepNs = 0, maxStepNs = 0, minStepNs = INT64_MAX;

	lastFrameTime = glfwGetTime();
	while (!glfwWindowShouldClose(mainWindow))
	{
		if (args.stepLimit && step > args.steps)
			break;

		double currentFrameTime = glfwGetTime();
		double frameSecs = currentFrameTime - lastFrameTime;
		lastFrameTime = currentFrameTime;
		frameSecsAccum += frameSecs;

		if (frameSecsAccum >= infoRefreshSecs)
		{
			avgFPS = frameCounter / frameSecsAccum;
			avgStepNs = stepNsAcum / frameCounter;

			frameCounter = 1;
			frameSecsAccum = 0.0;
			stepNsAcum = 0;
		}

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		ImGui_ImplOpenGL2_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		// Globally position text
		ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f));
		ImGui::Begin("Overlay", NULL, ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoScrollbar);
		ImGui::End();

		DrawText(5, 5, demoStrings[demoIndex]);
		DrawText(5, 35, "Keys: 1-9 Demos, Space to Launch the Bomb");

		char buffer[64];
		sprintf(buffer, "(A)ccumulation %s", World::accumulateImpulses ? "ON" : "OFF");
		DrawText(5, 65, buffer);

		sprintf(buffer, "(P)osition Correction %s", World::positionCorrection ? "ON" : "OFF");
		DrawText(5, 95, buffer);

		sprintf(buffer, "(W)arm Starting %s", World::warmStarting ? "ON" : "OFF");
		DrawText(5, 125, buffer);

		sprintf(buffer, "(G)ravity Enabled %s", world.gravity.y != 0.0f ? "ON" : "OFF");
		DrawText(5, 155, buffer);

		sprintf(buffer, "FPS: %2d", avgFPS);
		DrawText(5, 185, buffer);
		
		sprintf(buffer, "World Step Time:     %'14ld ns", avgStepNs);
		DrawText(5, 205, buffer);

		sprintf(buffer, "World Max Step Time: %'14ld ns", maxStepNs);
		DrawText(5, 225, buffer);

		sprintf(buffer, "World Min Step Time: %'14ld ns", minStepNs);
		DrawText(5, 245, buffer);

		sprintf(buffer, "Step: %lu", step);
		DrawText(5, 265, buffer);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();

		auto preStepTime = std::chrono::high_resolution_clock::now();
		world.Step(timeStep);
		auto posStepTime = std::chrono::high_resolution_clock::now();

		int64_t stepNs = std::chrono::nanoseconds(posStepTime - preStepTime).count();

		writeLogStepNs(logFile, stepNs);

		stepNsAcum += stepNs;
		if (maxStepNs < stepNs) maxStepNs = stepNs;
		if (minStepNs > stepNs) minStepNs = stepNs;

		for (int i = 0; i < numBodies; ++i)
			DrawBody(bodies + i);

		for (int i = 0; i < numJoints; ++i)
			DrawJoint(joints + i);

		glPointSize(4.0f);
		glColor3f(1.0f, 0.0f, 0.0f);
		glBegin(GL_POINTS);
		std::map<ArbiterKey, Arbiter>::const_iterator iter;
		for (iter = world.arbiters.begin(); iter != world.arbiters.end(); ++iter)
		{
			const Arbiter& arbiter = iter->second;
			for (int i = 0; i < arbiter.numContacts; ++i)
			{
				Vec2 p = arbiter.contacts[i].position;
				glVertex2f(p.x, p.y);
			}
		}
		glEnd();
		glPointSize(1.0f);

		ImGui::Render();
		ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

		glfwPollEvents();
		glfwSwapBuffers(mainWindow);

		frameCounter++;
		step++;
	}

	glfwTerminate();

	closeLogFile(logFile);

	return 0;
}
#endif

static int runDemoHeadless()
{
	FILE* logFile = openLogFile();
	int64_t avgStepNs = 0;
	int64_t stepM2 = 0;
	int64_t maxStepNs = 0;
	int64_t minStepNs = INT64_MAX;

	for (int step = 0; step < args.steps; ++step)
	{
		auto preStepTime = std::chrono::high_resolution_clock::now();
		world.Step(timeStep);
		auto posStepTime = std::chrono::high_resolution_clock::now();

		int64_t stepNs = std::chrono::nanoseconds(posStepTime - preStepTime).count();

		writeLogStepNs(logFile, stepNs);

		if (maxStepNs < stepNs) maxStepNs = stepNs;
		if (minStepNs > stepNs) minStepNs = stepNs;

		int64_t delta = stepNs - avgStepNs;
		avgStepNs += delta / (step + 1);
		int64_t delta2 = stepNs - avgStepNs;
		stepM2 += delta * delta2;
	}

	int64_t variance = (args.steps > 1) ? (stepM2 / (args.steps - 1)) : 0.0;
	int64_t stdStepNs = sqrt(variance);

	if (!args.log)
	{
		printf("Mean step time: %'14ld ns\n", avgStepNs);
		printf("Std step time:  %'14ld ns\n", stdStepNs);
		printf("Max step time:  %'14ld ns\n", maxStepNs);
		printf("Min step time:  %'14ld ns\n", minStepNs);
	}

	closeLogFile(logFile);

	return 0;
}

int main(int argc, char* const* argv)
{
	int exitCode;

	if (parseArgv(argc, argv) == -1)
		return 0;

	setlocale(LC_NUMERIC, "");

	const size_t&& sizeofMB = 1024 * 1024;
	const int&& numBodiesMargin = 10;

	maxBodies = MAX(args.numBodies + NUM_BODIES_MARGIN, MIN_NUM_BODIES);
	printf("Allocated bodies: %'lu MB\n", maxBodies * sizeof(Body) / sizeofMB);
	bodies = new Body[maxBodies];

	maxJoints = getMaxUsedJoints();
	printf("Allocated joints: %'lu MB\n", maxJoints * sizeof(Joint) / sizeofMB);
	joints = new Joint[maxJoints];

	InitDemo(args.demo);

#ifndef HEADLESS
	if (args.headless)
#endif
		exitCode = runDemoHeadless();
#ifndef HEADLESS
	else
		exitCode = runDemo();
#endif

	delete[] bodies;
	delete[] joints;

	return exitCode;
}

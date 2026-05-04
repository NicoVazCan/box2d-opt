/*
* Copyright (c) 2006-2009 Erin Catto http://www.gphysics.com
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Erin Catto makes no representations about the suitability 
* of this software for any purpose.  
* It is provided "as is" without express or implied warranty.
*/

#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <map>
#include "MathUtils.h"
#include <bvh.h>

struct Body;
struct Joint;

struct World
{
	World(Vec2 gravity, int iterations) : gravity(gravity), iterations(iterations) {}

	void Add(Body* body);
	void Add(Joint* joint);
	void Clear();

	void Step(float dt);

	void BroadPhase();

	bvh::bvh_t bodiesBVH;
	std::vector<Body*> bodies;
	std::vector<Joint*> joints;
	Vec2 gravity;
	int iterations;
#ifdef DEMO_TUNE
	static bool accumulateImpulses;
	static bool warmStarting;
	static bool positionCorrection;
#else
	static constexpr bool accumulateImpulses = true;
	static constexpr bool warmStarting = true;
	static constexpr bool positionCorrection = true;
#endif
};

#endif

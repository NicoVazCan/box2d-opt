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
#ifndef BOX2D_USE_ARBITER_MAP_PER_BODY
#	include "Arbiter.h"
#endif

#ifdef BOX2D_USE_BROADPHASE_BVH
#	include <bvh.h>
#elif defined(BOX2D_USE_BROADPHASE_SAP)
#	include <StdAfx.h>
#endif

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

#ifdef BOX2D_USE_BROADPHASE_BVH
	bvh::bvh_t bodiesBVH;
#elif defined(BOX2D_USE_BROADPHASE_SAP)
	std::vector<AABB> aabbBodies{{}};
#endif
	std::vector<Body*> bodies;
	std::vector<Joint*> joints;
#ifndef BOX2D_USE_ARBITER_MAP_PER_BODY
	std::map<ArbiterKey, Arbiter> arbiters;
#endif	
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

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

#include "box2d-lite/World.h"
#include "box2d-lite/Body.h"
#include "box2d-lite/Joint.h"
#ifdef BOX2D_USE_ARBITER_MAP_PER_BODY
#	include "box2d-lite/Arbiter.h"
#endif

#ifdef BOX2D_USE_OMP
#	include <omp.h>
#endif
#include <iostream>

using std::vector;
using std::map;
using std::pair;

#ifdef BOX2D_USE_ARBITER_MAP_PER_BODY
typedef map<Body*, Arbiter>::iterator ArbIter;
typedef pair<Body*, Arbiter> ArbPair;
#else
typedef map<ArbiterKey, Arbiter>::iterator ArbIter;
typedef pair<ArbiterKey, Arbiter> ArbPair;
#endif

#ifdef DEMO_TUNE
bool World::accumulateImpulses = true;
bool World::warmStarting = true;
bool World::positionCorrection = true;
#endif

void World::Add(Body* body)
{
#ifdef BOX2D_USE_BROADPHASE_BVH
	body->idxBodies = bodies.size();
	body->idxBVH = bodiesBVH.insert(body->GetAABB(), body);
#elif defined(BOX2D_USE_BROADPHASE_SAP)
	aabbBodies.emplace_back();
#endif
	bodies.push_back(body);
}

void World::Add(Joint* joint)
{
	joints.push_back(joint);
}

void World::Clear()
{
#ifdef BOX2D_USE_ARBITER_MAP_PER_BODY
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		Body* bi = bodies[i];
		bi->arbiters.clear();
	}
#endif
	bodies.clear();
#ifdef BOX2D_USE_BROADPHASE_BVH
	bodiesBVH.clear();
#elif defined(BOX2D_USE_BROADPHASE_SAP)
	aabbBodies.clear();
#endif
	joints.clear();
#ifndef BOX2D_USE_ARBITER_MAP_PER_BODY
	arbiters.clear();
#endif
}

void World::BroadPhase()
{
#ifdef BOX2D_USE_BROADPHASE_BVH
	std::vector<bvh::index_t> query;
#pragma omp for private(query) schedule(static) 
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		Body* bi = bodies[i];
		query.clear();
		bodiesBVH.find_overlaps(bi->GetAABB(), query);

		for (int j = 0; j < (int)query.size(); ++j)
		{
			bvh::index_t bjIdx = query[j];
			bvh::node_t bjNode = bodiesBVH.get(bjIdx);
			Body* bj = (Body*)bjNode.user_data;
		    
		    if (bj->idxBodies <= i) // Avoid checking pairs already checked
		    	continue;

			if (bi->invMass == 0.0f && bj->invMass == 0.0f)
				continue;

			Arbiter newArb(bi, bj);

			if (newArb.numContacts > 0)
			{
				ArbIter iter = bi->arbiters.find(bj);
				if (iter == bi->arbiters.end())
				{
					newArb.updated = true;
					bi->arbiters.insert(ArbPair(bj, newArb));
				}
				else
				{
					iter->second.updated = true;
					iter->second.Update(newArb.contacts, newArb.numContacts);
				}
			}
		}
	}
#elif defined(BOX2D_USE_BROADPHASE_SAP)
	static Container incs;
	static Container refs;

#pragma omp for schedule(static)
	for (int i = 0; i < bodies.size(); ++i)
	{
		AABB& aabb = aabbBodies[i];
		const Body* b = bodies[i];
		b->GetAABB(aabb);
	}

#pragma omp single
	{
		incs.Reset();
		refs.Reset();
		CompleteBoxPruning(bodies.size(), aabbBodies.data(), incs, refs);
	}

	int incsSize = incs.GetNbEntries();
#pragma omp for schedule(static)
	for (int i = 0; i < incsSize; i += 3)
	{
		udword iBody = incs[i];
		udword IdxStartRefs = incs[i + 1];
		udword IdxEndRefs = incs[i + 2];
		Body* bi = bodies[iBody];
		for (int j = IdxStartRefs; j < IdxEndRefs; ++j) {
			udword jBody = refs[j];
		    Body* bj = bodies[jBody];

			Arbiter newArb(bi, bj);

			if (newArb.numContacts > 0)
			{
				ArbIter iter = bi->arbiters.find(bj);
				if (iter == bi->arbiters.end())
				{
					newArb.updated = true;
					bi->arbiters.insert(ArbPair(bj, newArb));
				}
				else
				{
					iter->second.updated = true;
					iter->second.Update(newArb.contacts, newArb.numContacts);
				}
			}
		}
	}
#else
	// O(n^2) broad-phase
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		Body* bi = bodies[i];

		for (int j = i + 1; j < (int)bodies.size(); ++j)
		{
			Body* bj = bodies[j];

			if (bi->invMass == 0.0f && bj->invMass == 0.0f)
				continue;

			Arbiter newArb(bi, bj);

#	ifdef BOX2D_USE_ARBITER_MAP_PER_BODY
			if (newArb.numContacts > 0)
			{
				ArbIter iter = bi->arbiters.find(bj);
				if (iter == bi->arbiters.end())
				{
					newArb.updated = true;
					bi->arbiters.insert(ArbPair(bj, newArb));
				}
				else
				{
					iter->second.updated = true;
					iter->second.Update(newArb.contacts, newArb.numContacts);
				}
			}
#	else
			ArbiterKey key(bi, bj);

			if (newArb.numContacts > 0)
			{
				ArbIter iter = arbiters.find(key);
				if (iter == arbiters.end())
				{
					arbiters.insert(ArbPair(key, newArb));
				}
				else
				{
					iter->second.Update(newArb.contacts, newArb.numContacts);
				}
			}
			else
			{
				arbiters.erase(key);
			}
#	endif
		}
	}
#endif
}

#define BOX2D_MIN_BODIES_THREAD 10

void World::Step(float dt)
{
#pragma omp parallel if(bodies.size() > BOX2D_MIN_BODIES_THREAD)
	{
		float inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;

		// Determine overlapping bodies and update contact points.
		BroadPhase();

		// Integrate forces.
#pragma omp for schedule(static) 
		for (int i = 0; i < (int)bodies.size(); ++i)
		{
			Body* b = bodies[i];

			if (b->invMass != 0.0f)
			{
				b->velocity += dt * (gravity + b->invMass * b->force);
				b->angularVelocity += dt * b->invI * b->torque;
			}
		}

		// Perform pre-steps.
#pragma omp for schedule(static)
#ifdef BOX2D_USE_ARBITER_MAP_PER_BODY
		for (int i = 0; i < (int)bodies.size(); ++i)
		{
			Body* bi = bodies[i];
			for (ArbIter arb = bi->arbiters.begin(); arb != bi->arbiters.end();)
			{
				if (arb->second.updated)
				{
					arb->second.PreStep(inv_dt);
					arb->second.updated = false;
					++arb;
				}
				else
				{
					// If there have been no touchpoints in the current frame, delete
					arb = bi->arbiters.erase(arb);
				}
			}
		}
#else
		for (ArbIter arb = arbiters.begin(); arb != arbiters.end(); ++arb)
		{
			arb->second.PreStep(inv_dt);
		}
#endif
#pragma omp for schedule(static)
		for (int i = 0; i < (int)joints.size(); ++i)
		{
			joints[i]->PreStep(inv_dt);	
		}

		// Perform iterations
		for (int i = 0; i < iterations; ++i)
		{
#pragma omp for private(i) schedule(static)
#ifdef BOX2D_USE_ARBITER_MAP_PER_BODY
			for (int j = 0; j < (int)bodies.size(); ++j)
			{
				Body* bi = bodies[j];
				for (ArbIter arb = bi->arbiters.begin(); arb != bi->arbiters.end(); ++arb)
				{
					arb->second.ApplyImpulse();
				}
			}
#else
			for (ArbIter arb = arbiters.begin(); arb != arbiters.end(); ++arb)
			{
				arb->second.ApplyImpulse();
			}
#endif
#pragma omp for private(i) schedule(static)
			for (int j = 0; j < (int)joints.size(); ++j)
			{
				joints[j]->ApplyImpulse();
			}
		}

		// Integrate Velocities
#pragma omp for schedule(static)
		for (int i = 0; i < (int)bodies.size(); ++i)
		{
			Body* b = bodies[i];

			b->position += dt * b->velocity;
			b->rotation += dt * b->angularVelocity;

			b->force.Set(0.0f, 0.0f);
			b->torque = 0.0f;
		}
	}

#ifdef BOX2D_USE_BROADPHASE_BVH
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		Body* b = bodies[i];
		bodiesBVH.move(b->idxBVH, b->GetAABB());
	}
#endif
}

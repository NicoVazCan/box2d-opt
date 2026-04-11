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
#include "box2d-lite/Arbiter.h"

#include <omp.h>
#include <iostream>

using std::vector;
using std::map;
using std::pair;

typedef map<ArbiterKey, Arbiter>::iterator ArbIter;
typedef pair<ArbiterKey, Arbiter> ArbPair;

bool World::accumulateImpulses = true;
bool World::warmStarting = true;
bool World::positionCorrection = true;

void World::Add(Body* body)
{
	body->idxWorld = bodies.size();
	bodies.push_back(body);
	body->idxBVH = bodiesBVH.insert(body->GetAABB(), body);
}

void World::Add(Joint* joint)
{
	joints.push_back(joint);
}

void World::Clear()
{
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		Body* bi = bodies[i];
		bi->arbiters.clear();
	}
	bodies.clear();
	bodiesBVH.clear();
	joints.clear();
}

void World::BroadPhase()
{
	std::vector<bvh::index_t> query;
#pragma omp parallel for private(query) shared(bodies, bodiesBVH) schedule(dynamic,512) if(bodies.size() >= 512)
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
		    
		    if (bj->idxWorld <= i) // Avoid checking pairs already checked
		    	continue;

			if (bi->invMass == 0.0f && bj->invMass == 0.0f)
				continue;

			Arbiter newArb(bi, bj);
			ArbiterKey key(bi, bj);

			if (newArb.numContacts > 0)
			{
				ArbIter iter = bi->arbiters.find(key);
				if (iter == bi->arbiters.end())
				{
					newArb.updated = true;
					bi->arbiters.insert(ArbPair(key, newArb));
				}
				else
				{
					iter->second.Update(newArb.contacts, newArb.numContacts);
					iter->second.updated = true;
				}
			}
		}
	}
}

void World::Step(float dt)
{
	float inv_dt = dt > 0.0f ? 1.0f / dt : 0.0f;

	// Determine overlapping bodies and update contact points.
	BroadPhase();

	// Integrate forces.
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		Body* b = bodies[i];

		if (b->invMass == 0.0f)
			continue;

		b->velocity += dt * (gravity + b->invMass * b->force);
		b->angularVelocity += dt * b->invI * b->torque;
	}

	// Perform pre-steps.
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

	for (int i = 0; i < (int)joints.size(); ++i)
	{
		joints[i]->PreStep(inv_dt);	
	}

	// Perform iterations
#pragma omp parallel shared(bodies) if(bodies.size() >= 512)
	for (int i = 0; i < iterations; ++i)
	{
#pragma omp for private(i) schedule(dynamic, 512)
		for (int j = 0; j < (int)bodies.size(); ++j)
		{
			Body* bi = bodies[j];
			for (ArbIter arb = bi->arbiters.begin(); arb != bi->arbiters.end(); ++arb)
			{
				arb->second.ApplyImpulse();
			}
		}
#pragma omp for private(i) schedule(dynamic, 512)
		for (int j = 0; j < (int)joints.size(); ++j)
		{
			joints[j]->ApplyImpulse();
		}
	}

	// Integrate Velocities
#pragma omp parallel for shared(bodies, bodiesBVH)
	for (int i = 0; i < (int)bodies.size(); ++i)
	{
		Body* b = bodies[i];

		b->position += dt * b->velocity;
		b->rotation += dt * b->angularVelocity;

		b->force.Set(0.0f, 0.0f);
		b->torque = 0.0f;

		bodiesBVH.move(b->idxBVH, b->GetAABB());
	}
}

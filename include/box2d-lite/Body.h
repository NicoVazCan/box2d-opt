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

#ifndef BODY_H
#define BODY_H

#include <map>
#include "MathUtils.h"
#include "Arbiter.h"
#include <bvh.h>

struct Body
{
	Body();
	void Set(const Vec2& w, float m);

	void AddForce(const Vec2& f)
	{
		force += f;
	}

	bvh::aabb_t GetAABB()
	{
	    float c = cosf(rotation);
	    float s = sinf(rotation);

	    Vec2 half = width;
	    half *= 0.5f;

	    // Compute rotated extents
	    float extentX = fabs(half.x * c) + fabs(half.y * s);
	    float extentY = fabs(half.x * s) + fabs(half.y * c);

	    Vec2 extents(extentX, extentY);
	    Vec2 min = position - extents;
	    Vec2 max = position + extents;
	    bvh::aabb_t aabb{min.x, min.y, max.x, max.y};

	    return aabb;
	}

	Vec2 position;
	float rotation;

	Vec2 velocity;
	float angularVelocity;

	Vec2 force;
	float torque;

	Vec2 width;

	float friction;
	float mass, invMass;
	float I, invI;

	std::map<Body*, Arbiter> arbiters;
	bvh::index_t idxBVH;
	int idxWorld;
};

#endif

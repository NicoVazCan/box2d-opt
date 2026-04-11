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

#include "box2d-lite/Joint.h"
#include "box2d-lite/Body.h"
#include "box2d-lite/World.h"

void Joint::Set(Body* b1, Body* b2, const Vec2& anchor)
{
	body1 = b1;
	body2 = b2;

	Mat22 Rot1(body1->rotation);
	Mat22 Rot2(body2->rotation);
	Mat22 Rot1T = Rot1.Transpose();
	Mat22 Rot2T = Rot2.Transpose();

	localAnchor1 = Rot1T * (anchor - body1->position);
	localAnchor2 = Rot2T * (anchor - body2->position);

	P.Set(0.0f, 0.0f);

	softness = 0.0f;
	biasFactor = 0.2f;
}

void Joint::PreStep(float inv_dt)
{
	// Pre-compute anchors, mass matrix, and bias.
	Mat22 Rot1(body1->rotation);
	Mat22 Rot2(body2->rotation);

	r1 = Rot1 * localAnchor1;
	r2 = Rot2 * localAnchor2;

	// deltaV = deltaV0 + K * impulse
	// invM = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	Mat22 K1;
	K1.col1.x = body1->invMass + body2->invMass;	K1.col2.x = 0.0f;
	K1.col1.y = 0.0f;								K1.col2.y = body1->invMass + body2->invMass;

	Mat22 K2;
	K2.col1.x =  body1->invI * r1.y * r1.y;		K2.col2.x = -body1->invI * r1.x * r1.y;
	K2.col1.y = -body1->invI * r1.x * r1.y;		K2.col2.y =  body1->invI * r1.x * r1.x;

	Mat22 K3;
	K3.col1.x =  body2->invI * r2.y * r2.y;		K3.col2.x = -body2->invI * r2.x * r2.y;
	K3.col1.y = -body2->invI * r2.x * r2.y;		K3.col2.y =  body2->invI * r2.x * r2.x;

	Mat22 K = K1 + K2 + K3;
	K.col1.x += softness;
	K.col2.y += softness;

	M = K.Invert();

	Vec2 p1 = body1->position + r1;
	Vec2 p2 = body2->position + r2;
	Vec2 dp = p2 - p1;

	if (World::positionCorrection)
	{
		bias = -biasFactor * inv_dt * dp;
	}
	else
	{
		bias.Set(0.0f, 0.0f);
	}

	if (World::warmStarting)
	{
		// Apply accumulated impulse.
		body1->velocity -= body1->invMass * P;
		body1->angularVelocity -= body1->invI * Cross(r1, P);

		body2->velocity += body2->invMass * P;
		body2->angularVelocity += body2->invI * Cross(r2, P);
	}
	else
	{
		P.Set(0.0f, 0.0f);
	}
}

void Joint::ApplyImpulse()
{
	Vec2 b1Velocity, b2Velocity;
	float b1AngularVelocity, b2AngularVelocity;
#pragma omp atomic read
	b1Velocity.xy = body1->velocity.xy;
#pragma omp atomic read
	b2Velocity.xy = body2->velocity.xy;

#pragma omp atomic read
	b1AngularVelocity = body1->angularVelocity;
#pragma omp atomic read
	b2AngularVelocity = body2->angularVelocity;

    Vec2 dv = b2Velocity + Cross(b2AngularVelocity, r2) - b1Velocity - Cross(b1AngularVelocity, r1);

	Vec2 impulse;

	impulse = M * (bias - dv - softness * P);

	b1Velocity -= body1->invMass * impulse;
	b1AngularVelocity -= body1->invI * Cross(r1, impulse);

	b2Velocity += body2->invMass * impulse;
	b2AngularVelocity += body2->invI * Cross(r2, impulse);

	P += impulse;

#pragma omp atomic write
	body1->velocity.xy = b1Velocity.xy;
#pragma omp atomic write
	body2->velocity.xy = b2Velocity.xy;

#pragma omp atomic write
	body1->angularVelocity = b1AngularVelocity;
#pragma omp atomic write
	body2->angularVelocity = b2AngularVelocity;
}

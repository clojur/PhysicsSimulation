//
//  Scene.cpp
//
#include "Scene.h"
#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

/*
========================================================================================================

Scene

========================================================================================================
*/

/*
====================================================
Scene::~Scene
====================================================
*/
Scene::~Scene() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();
}

/*
====================================================
Scene::Reset
====================================================
*/
void Scene::Reset() {
	for ( int i = 0; i < m_bodies.size(); i++ ) {
		delete m_bodies[ i ].m_shape;
	}
	m_bodies.clear();

	Initialize();
}

/*
====================================================
Scene::Initialize
====================================================
*/
void Scene::Initialize()
{
	Body body;
	body.m_position = Vec3( 0, 0, 10.0f );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_shape = new ShapeSphere( 0.05f );
	m_bodies.push_back( body );


	body.m_position = Vec3(0, 0, 9.6f);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_shape = new ShapeSphere(0.05f);
	m_bodies.push_back(body);

	body.m_position = Vec3(0, 0, 9.2f);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_shape = new ShapeSphere(0.05f);
	m_bodies.push_back(body);

	body.m_position = Vec3(0, 0, 8.8f);
	body.m_orientation = Quat(0, 0, 0, 1);
	body.m_shape = new ShapeSphere(0.05f);
	m_bodies.push_back(body);

	body.m_position = Vec3( 0, 0, -1001 );
	body.m_orientation = Quat( 0, 0, 0, 1 );
	body.m_shape = new ShapeSphere( 1000.0f );
	m_bodies.push_back( body );


	// TODO: Add code
}

/*
====================================================
Scene::Update
====================================================
*/
void Scene::Update( const float dt_sec )
{
	// TODO: Add code
	double timeStep = 0.01;
	double temp_dt = timeStep;
	float g = 9.8f;
	float m = 0.1f;

	float damping = 0.98f;
	std::vector<Vec3> tempVelocitys, tempPositions;
	for(int i = 0; i < m_bodies.size() - 1; ++i)
	{
		tempVelocitys.emplace_back(m_bodies[i].m_velocity);
		tempPositions.emplace_back(m_bodies[i].m_position);
	}

	//update position
	if(temp_dt < dt_sec)
	{
		while(temp_dt < dt_sec)
		{
			for (int i = 0; i < m_bodies.size() - 1; ++i)
			{
				tempVelocitys[i] += Vec3(0, 0, -g) * timeStep * damping;
				tempPositions[i] += tempVelocitys[i] * timeStep * damping;
			}
			temp_dt += timeStep;
		}
	}
	else
	{
		for (int i = 0; i < m_bodies.size() - 1; ++i)
		{
			tempVelocitys[i] += Vec3(0, 0, -g) * timeStep * damping;
			tempPositions[i] += tempVelocitys[i] * timeStep * damping;
		}
	}

	//collision test
	int groundIdx = m_bodies.size() - 1;
	Vec3 gPos = m_bodies[groundIdx].m_position;
	float gRadius = dynamic_cast<ShapeSphere*>(m_bodies[groundIdx].m_shape)->m_radius;
	float epsilon = 0.005f;
	for (int i = 0; i < groundIdx; ++i)
	{
		Vec3 groundToSphere = tempPositions[i] - gPos;
		
		float disFun= groundToSphere.GetLengthSqr()- (gRadius * gRadius);

		//·¢ÉúÅö×²
		if(disFun <= epsilon)
		{
			float r = dynamic_cast<ShapeSphere*>(m_bodies[groundIdx].m_shape)->m_radius;

			m_bodies[i].m_position = gPos+(groundToSphere.Normalize()*(gRadius+0.05f));
			m_bodies[i].m_velocity = Vec3();
		}
		else
		{
			m_bodies[i].m_position = tempPositions[i];
			m_bodies[i].m_velocity = tempVelocitys[i];
		}
	}


}
#pragma once

#include "Jolt/Jolt.h"
#include "Jolt/Physics/Body/Body.h"
#include "Jolt/Physics/Body/BodyCreationSettings.h"
#include "Jolt/Physics/Collision/Shape/BoxShape.h"
#include "Jolt/Physics/Collision/Shape/MeshShape.h"
#include "Jolt/Physics/PhysicsSystem.h"
#include "Utilities.h"

struct Terrain
{
	JPH::Body* physicsTerrain;
};

class TerrainHandler
{
public:
	TerrainHandler(JPH::PhysicsSystem& ph);
private:
	Terrain* GenerateTerrain(JPH::PhysicsSystem& ph);

	Terrain* mTerrain;
};

#pragma once

#include "Jolt/Jolt.h"
#include "Jolt/Physics/Body/Body.h"
#include "Jolt/Physics/Body/BodyCreationSettings.h"
#include "Jolt/Physics/Collision/Shape/BoxShape.h"
#include "Jolt/Physics/Collision/Shape/MeshShape.h"
#include "Jolt/Physics/PhysicsSystem.h"
#include "raylib.h"
#include "Utilities.h"

struct Terrain
{
	JPH::Body* physicsTerrain;
	Mesh* meshTerrain;
};

class TerrainHandler
{
public:
	TerrainHandler(JPH::PhysicsSystem& ph);
	void DrawTerrain();
private:
	Terrain* GenerateTerrain(JPH::PhysicsSystem& ph);
	void CreateTerrainTriangle(Mesh& mesh, JPH::Float3 v0, JPH::Float3 v1, JPH::Float3 v2, int vertexIndex, int texCoordIndex);

	Terrain* mTerrain;
	Model mTerrainModel;
};

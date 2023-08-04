#include "TerrainHandler.h"
#include "Utilities.h"

#include <iostream>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#define CELL_SIZE 10.0f
#define MAX_HEIGHT 250.0f

TerrainHandler::TerrainHandler(JPH::PhysicsSystem& ph)
{
    mTerrain = GenerateTerrain(ph);
}


Terrain* TerrainHandler::GenerateTerrain(JPH::PhysicsSystem& ph)
{
    int width, height, channels;
    unsigned char* image = stbi_load("terrain.png", &width, &height, &channels, STBI_grey);
    if (!image)
    {
        std::cerr << "Failed to load image." << std::endl;
    }
    if (height != width)
    {
        std::cerr << "Only square textures are supported." << std::endl;
    }
    
    int n = height;

   std::vector<std::vector<float>> heightmap(n, std::vector<float>(n));

    for (int x = 0; x < n; ++x)
    {
        for (int z = 0; z < n; ++z)
        {
            unsigned char r = image[x * n + z];
            heightmap[x][z] = static_cast<float>(r) / 256.0f * MAX_HEIGHT;
        }
    }

    stbi_image_free(image);

    n /= 4;

    // Possible improvement: use a HeightFieldTerrain

    JPH::TriangleList physTriangles;
    std::vector<OBJUtil::Vertex> meshVertices;
    std::vector<OBJUtil::UV> meshUVs;
    std::vector<OBJUtil::Normal> meshNormals;
    std::vector<OBJUtil::Triangle> meshTriangles;
    int vertexIndex = 0;

	meshUVs.push_back(OBJUtil::UV{ 0.0f, 0.0f });
	meshUVs.push_back(OBJUtil::UV{ 1.0f, 0.0f });
	meshUVs.push_back(OBJUtil::UV{ 0.0f, 1.0f });
    meshUVs.push_back(OBJUtil::UV{ 1.0f, 1.0f });

	meshNormals.push_back(OBJUtil::Normal{ 0.0f, 1.0f, 0.0f });
    
    float center = n * CELL_SIZE / 2.0f;
    for (int x = 0; x < n - 1; ++x)
    {
        for (int z = 0; z < n - 1; ++z)
        {
            float x1 = CELL_SIZE * x - center;
            float z1 = CELL_SIZE * z - center;
            float x2 = x1 + CELL_SIZE;
            float z2 = z1 + CELL_SIZE;

            JPH::Float3 v1p = JPH::Float3(x1, heightmap[x][z], z1);
            JPH::Float3 v2p = JPH::Float3(x2, heightmap[x + 1][z], z1);
            JPH::Float3 v3p = JPH::Float3(x1, heightmap[x][z + 1], z2);
            JPH::Float3 v4p = JPH::Float3(x2, heightmap[x + 1][z + 1], z2);

            JPH::Triangle t1p = JPH::Triangle(v1p, v3p, v4p);
            JPH::Triangle t2p = JPH::Triangle(v1p, v4p, v2p);

            physTriangles.push_back(t1p);
            physTriangles.push_back(t2p);

            OBJUtil::Vertex v1m = OBJUtil::Vertex{ x1, static_cast<float>(heightmap[x][z]), z1 };
			OBJUtil::Vertex v2m = OBJUtil::Vertex{ x2, static_cast<float>(heightmap[x + 1][z]), z1 };
			OBJUtil::Vertex v3m = OBJUtil::Vertex{ x1, static_cast<float>(heightmap[x][z + 1]), z2 };
			OBJUtil::Vertex v4m = OBJUtil::Vertex{ x2, static_cast<float>(heightmap[x + 1][z + 1]), z2 };

            meshVertices.push_back(v1m);
            meshVertices.push_back(v2m);
            meshVertices.push_back(v3m);
            meshVertices.push_back(v4m);

			OBJUtil::Triangle t1m = OBJUtil::Triangle{ 
                vertexIndex, vertexIndex + 2, vertexIndex + 3, 
                0, 2, 3,
                0, 0, 0
            };
			OBJUtil::Triangle t2m = OBJUtil::Triangle{ 
                vertexIndex, vertexIndex + 3, vertexIndex + 1,
                0, 3, 1,
                0, 0, 0
            };

			meshTriangles.push_back(t1m);
			meshTriangles.push_back(t2m);

            vertexIndex += 4;
        }
    }

    OBJUtil::OBJExporter::SaveToObj(meshVertices, meshUVs, meshNormals, meshTriangles, "media/terrain.obj");

    // Floor
    JPH::Body* floor = ph.GetBodyInterface().CreateBody(JPH::BodyCreationSettings(new JPH::MeshShapeSettings(physTriangles), JPH::RVec3::sZero(), JPH::Quat::sIdentity(), JPH::EMotionType::Static, JPHUtil::NON_MOVING));
    floor->SetFriction(1.0f);
    ph.GetBodyInterface().AddBody(floor->GetID(), JPH::EActivation::Activate);

    Terrain generatedTerrain = { floor };
    return &generatedTerrain;
}
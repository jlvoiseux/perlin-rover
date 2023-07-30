#include "TerrainHandler.h"

#include <iostream>

//#define STB_IMAGE_IMPLEMENTATION
//#include "stb_image.h"

#define TERRAIN_WORLD_SIZE 1000.0f

TerrainHandler::TerrainHandler(JPH::PhysicsSystem& ph)
{
    mTerrain = GenerateTerrain(ph);
    //mTerrainModel = LoadModelFromMesh(*mTerrain->meshTerrain);
}


Terrain* TerrainHandler::GenerateTerrain(JPH::PhysicsSystem& ph)
{
    /*int width, height, channels;
    unsigned char* image = stbi_load("models/terrain.png", &width, &height, &channels, STBI_grey);
    if (!image)
    {
        std::cerr << "Failed to load image." << std::endl;
    }
    if (height != width)
    {
        std::cerr << "Only square textures are supported." << std::endl;
    }
    stbi_image_free(image);*/

   // Image image = LoadImage("terrain.png");
   // int width = image.width;
   // int height = image.height;
   // if (height != width)
   // {
   //     std::cerr << "Only square textures are supported." << std::endl;
   // }
   // 
   // int n = height / 16;

   // std::vector<std::vector<unsigned char>> heightmap(n, std::vector<unsigned char>(n));

   // for (int x = 0; x < n; ++x)
   // {
   //     for (int z = 0; z < n; ++z)
   //     {
   //         /*unsigned char r = image[x * n + z];*/
   //         Color pixelColor = GetImageColor(image, x, z);
   //         heightmap[x][z] = pixelColor.r;
   //     }
   // }

   // const float cellSize = TERRAIN_WORLD_SIZE / n;
   // int renderTriangleCount = (n - 1) * (n - 1) * 2;
   // int vertexIndex = 0;
   // int texCoordIndex = 0;

   // Mesh mesh = { 0 };
   // mesh.triangleCount = renderTriangleCount;
   // mesh.vertexCount = mesh.triangleCount * 3;
   // mesh.vertices = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));
   // mesh.texcoords = (float*)MemAlloc(mesh.vertexCount * 2 * sizeof(float));
   // mesh.normals = (float*)MemAlloc(mesh.vertexCount * 3 * sizeof(float));

   // // Possible improvement: use a HeightFieldTerrain

   // JPH::TriangleList triangles;
   // float center = n * cellSize / 2.0f;
   // for (int x = 0; x < n - 1; ++x)
   // {
   //     for (int z = 0; z < n - 1; ++z)
   //     {
   //         float x1 = cellSize * x - center;
   //         float z1 = cellSize * z - center;
   //         float x2 = x1 + cellSize;
   //         float z2 = z1 + cellSize;

   //         JPH::Float3 v1 = JPH::Float3(x1, heightmap[x][z], z1);
   //         JPH::Float3 v2 = JPH::Float3(x2, heightmap[x + 1][z], z1);
   //         JPH::Float3 v3 = JPH::Float3(x1, heightmap[x][z + 1], z2);
   //         JPH::Float3 v4 = JPH::Float3(x2, heightmap[x + 1][z + 1], z2);
   //         
			//JPH::Triangle t1 = JPH::Triangle(v1, v3, v4);
			//JPH::Triangle t2 = JPH::Triangle(v1, v4, v2);

   //         triangles.push_back(t1);
   //         triangles.push_back(t2);

   //         CreateTerrainTriangle(mesh, v1, v3, v4, vertexIndex, texCoordIndex);
   //         CreateTerrainTriangle(mesh, v1, v4, v2, vertexIndex, texCoordIndex);
   //         vertexIndex += 18;
   //         texCoordIndex += 12;
   //     }
   // }

   // UploadMesh(&mesh, false);

   // // Floor
   // JPH::Body* floor = ph.GetBodyInterface().CreateBody(JPH::BodyCreationSettings(new JPH::MeshShapeSettings(triangles), JPH::RVec3::sZero(), JPH::Quat::sIdentity(), JPH::EMotionType::Static, JPHUtil::NON_MOVING));
   // floor->SetFriction(1.0f);
   // ph.GetBodyInterface().AddBody(floor->GetID(), JPH::EActivation::Activate);

   // Terrain generatedTerrain = { floor, &mesh };
   // return &generatedTerrain;
    return nullptr;
}


//void TerrainHandler::CreateTerrainTriangle(Mesh& mesh, JPH::Float3 v0, JPH::Float3 v1, JPH::Float3 v2, int vertexIndex, int texCoordIndex)
//{
//    mesh.vertices[vertexIndex] = v0.x;
//    mesh.vertices[vertexIndex + 1] = v0.y;
//    mesh.vertices[vertexIndex + 2] = v0.z;
//    mesh.normals[vertexIndex] = 0;
//    mesh.normals[vertexIndex + 1] = 1;
//    mesh.normals[vertexIndex + 2] = 0;
//    mesh.texcoords[texCoordIndex] = 0;
//    mesh.texcoords[texCoordIndex + 1] = 0;
//
//    // Vertex at (1, 0, 2)
//    mesh.vertices[vertexIndex + 3] = v1.x;
//    mesh.vertices[vertexIndex + 4] = v1.y;
//    mesh.vertices[vertexIndex + 5] = v1.z;
//    mesh.normals[vertexIndex + 3] = 0;
//    mesh.normals[vertexIndex + 4] = 1;
//    mesh.normals[vertexIndex + 5] = 0;
//    mesh.texcoords[texCoordIndex + 2] = 0.5f;
//    mesh.texcoords[texCoordIndex + 3] = 1.0f;
//
//    // Vertex at (2, 0, 0)
//    mesh.vertices[vertexIndex + 6] = v2.x;
//    mesh.vertices[vertexIndex + 7] = v2.y;
//    mesh.vertices[vertexIndex + 8] = v2.z;
//    mesh.normals[vertexIndex + 6] = 0;
//    mesh.normals[vertexIndex + 7] = 1;
//    mesh.normals[vertexIndex + 8] = 0;
//    mesh.texcoords[texCoordIndex + 4] = 1;
//    mesh.texcoords[texCoordIndex + 5] = 0;
//}


void TerrainHandler::DrawTerrain()
{
    //DrawModel(mTerrainModel, Vector3{ 0.0f, 0.0f, 0.0f }, 1.0f, WHITE);
}
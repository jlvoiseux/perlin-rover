#pragma once
#include <iostream>
#include <cstdarg>
#include <thread>

#include "Falcor.h"
#include "Jolt/Jolt.h"
#include "Jolt/Core/Factory.h"
#include "Jolt/Core/JobSystemThreadPool.h"
#include "Jolt/RegisterTypes.h"
#include "KeyHandler.h"
#include "Renderer.h"
#include "TerrainHandler.h"
#include "VehicleHandler.h"


int main(int argc, char** argv)
{
    SampleAppConfig config;
    config.windowDesc.title = "HelloDXR";
    config.windowDesc.resizableWindow = true;

    Renderer renderer(config);
    return renderer.run();
}

//int main(int argc, char* argv[])
//{
//    const int screenWidth = 800;
//    const int screenHeight = 450;
//    InitWindow(screenWidth, screenHeight, "Perlin Rover");
//
//    // PHYSICS
//    JPH::RegisterDefaultAllocator();
//    JPH::Factory::sInstance = new JPH::Factory();
//    JPH::RegisterTypes();
//    JPH::TempAllocatorImpl tempAllocator(10 * 1024 * 1024);
//    JPH::JobSystemThreadPool jobSystem(2048, 8, std::thread::hardware_concurrency() - 1);
//    JPHUtil::BPLayerInterfaceImpl mBroadPhaseLayerInterface;
//    JPHUtil::ObjectVsBroadPhaseLayerFilterImpl mObjectVsBroadPhaseLayerFilter;
//    JPHUtil::ObjectLayerPairFilterImpl mObjectVsObjectLayerFilter;
//    JPH::PhysicsSystem physics;
//    physics.Init(65536, 0, 1024, 1024, mBroadPhaseLayerInterface, mObjectVsBroadPhaseLayerFilter, mObjectVsObjectLayerFilter);
//    VehicleHandler vehicle = VehicleHandler(physics);
//
//    // TERRAIN
//    TerrainHandler terrainHandler(physics);
//
//    // CAMERA
//    Camera camera = { 0 };
//    camera.position = Vector3{ 10.0f, 10.0f, 10.0f }; // Camera position
//    camera.target = Vector3{ 0.0f, 0.0f, 0.0f };      // Camera looking at point
//    camera.up = Vector3{ 0.0f, 1.0f, 0.0f };          // Camera up vector (rotation towards target)
//    camera.fovy = 45.0f;                                // Camera field-of-view Y
//    camera.projection = CAMERA_PERSPECTIVE;             // Camera projection type
//
//    KeyHandler keyHandler;
//
//    static constexpr float deltaTime = 1.0f / 60.0f;
//    SetTargetFPS(60);
//    while (!WindowShouldClose())
//    {
//        keyHandler.update();
//        vehicle.Update(physics, keyHandler.Fwd, keyHandler.Bwd, keyHandler.Lft, keyHandler.Rht);
//        physics.Update(deltaTime, 1, &tempAllocator, &jobSystem);
//
//        JPH::RVec3 wheelFwdLeftPos = vehicle.GetWheel(VehicleHandler::EWheel::FrontLeft)->GetBody2()->GetPosition();
//        JPH::Quat wheelFwdLeftRot = vehicle.GetWheel(VehicleHandler::EWheel::FrontLeft)->GetBody2()->GetRotation();
//
//
//        JPH::RVec3 wheelFwdRightPos = vehicle.GetWheel(VehicleHandler::EWheel::FrontRight)->GetBody2()->GetPosition();
//        JPH::Quat wheelFwdRightRot = vehicle.GetWheel(VehicleHandler::EWheel::FrontRight)->GetBody2()->GetRotation();
//
//
//        JPH::RVec3 wheelRearLeftPos = vehicle.GetWheel(VehicleHandler::EWheel::RearLeft)->GetBody2()->GetPosition();
//        JPH::Quat wheelRearLeftRot = vehicle.GetWheel(VehicleHandler::EWheel::RearLeft)->GetBody2()->GetRotation();
//
//
//        JPH::RVec3 wheelRearRightPos = vehicle.GetWheel(VehicleHandler::EWheel::RearRight)->GetBody2()->GetPosition();
//        JPH::Quat wheelRearRightRot = vehicle.GetWheel(VehicleHandler::EWheel::RearRight)->GetBody2()->GetRotation();
//
//
//        JPH::RVec3 chassisPos = vehicle.GetChassis()->GetPosition();
//        JPH::Quat chassisRot = vehicle.GetChassis()->GetRotation();
//		
//
//		BeginDrawing();
//        ClearBackground(BLACK);
//        BeginMode3D(camera);
//        terrainHandler.DrawTerrain();
//		vehicle.DrawVehicle(wheelFwdLeftPos, wheelFwdLeftRot, wheelFwdRightPos, wheelFwdRightRot, wheelRearLeftPos, wheelRearLeftRot, wheelRearRightPos, wheelRearRightRot, chassisPos, chassisRot);
//        
//        camera.position = vehicle.cameraPositionAnchorPoint;
//        camera.target = vehicle.cameraTargetAnchorPoint;
//
//        EndMode3D();
//        EndDrawing();
//    }
//
//    CloseWindow();
//    return 0;
//}
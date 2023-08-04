#pragma once

#include <iostream>
#include <cstdarg>
#include <thread>
#include <chrono>

#include "Falcor.h"
#include "Core/SampleApp.h"
#include "Core/Pass/RasterPass.h"
#include "Utils/Math/FalcorMath.h"
#include "Utils/UI/TextRenderer.h"
#include "Jolt/Jolt.h"
#include "Jolt/Core/Factory.h"
#include "Jolt/Core/JobSystemThreadPool.h"
#include "Jolt/RegisterTypes.h"

#include "KeyHandler.h"
#include "TerrainHandler.h"
#include "VehicleHandler.h"

class Renderer : public Falcor::SampleApp
{
public:
    Renderer(const Falcor::SampleAppConfig& config);
    ~Renderer();

    void onLoad(Falcor::RenderContext* pRenderContext) override;
    void onResize(uint32_t width, uint32_t height) override;
    void onFrameRender(Falcor::RenderContext* pRenderContext, const Falcor::ref<Falcor::Fbo>& pTargetFbo) override;
    bool onKeyEvent(const Falcor::KeyboardEvent& keyEvent) override;

    void UpdateMesh(int nodeID, Falcor::Transform transform);

private:
    void loadScene(const std::filesystem::path& path, const Falcor::Fbo* pTargetFbo);
    void setPerFrameVars(const Falcor::Fbo* pTargetFbo);
    void renderRaster(Falcor::RenderContext* pRenderContext, const Falcor::ref<Falcor::Fbo>& pTargetFbo);
    void renderRT(Falcor::RenderContext* pRenderContext, const Falcor::ref<Falcor::Fbo>& pTargetFbo);

    Falcor::ref<Falcor::Scene> mpScene;
    Falcor::ref<Falcor::Camera> mpCamera;

    Falcor::ref<Falcor::RasterPass> mpRasterPass;

    Falcor::ref<Falcor::RtProgram> mpRaytraceProgram;
    Falcor::ref<Falcor::RtProgramVars> mpRtVars;
    Falcor::ref<Falcor::Texture> mpRtOut;

    bool mRayTrace = true;
    bool mUseDOF = false;

    uint32_t mSampleIndex = 0xdeadbeef;
    
    std::chrono::steady_clock::time_point mPreviousTime;
    bool mFirstFrame;

    KeyHandler mKeyHandler;
    
    JPH::PhysicsSystem* mPhysics;
    JPH::TempAllocatorImpl* mTempAllocator;
    JPH::JobSystemThreadPool* mJobSystem;
    JPHUtil::BPLayerInterfaceImpl* mBroadPhaseLayerInterface;
    JPHUtil::ObjectVsBroadPhaseLayerFilterImpl* mObjectVsBroadPhaseLayerFilter;
    JPHUtil::ObjectLayerPairFilterImpl* mObjectVsObjectLayerFilter;
    VehicleHandler* mVehicle;
};

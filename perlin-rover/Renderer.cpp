#include "Renderer.h"

#include "Scene/Animation/AnimationController.h"

static const Falcor::float4 kClearColor(0.93f, 0.73f, 0.83f, 1.f);
static const std::string kDefaultScene = "scene.pyscene";

Renderer::Renderer(const Falcor::SampleAppConfig& config) : SampleApp(config) {}

Renderer::~Renderer() 
{
	delete mVehicle;
	delete mTempAllocator;
	delete mJobSystem;
	delete mBroadPhaseLayerInterface;
	delete mObjectVsBroadPhaseLayerFilter;
	delete mObjectVsObjectLayerFilter;
    delete mPhysics;
}

void Renderer::onLoad(Falcor::RenderContext* pRenderContext)
{
    if (getDevice()->isFeatureSupported(Falcor::Device::SupportedFeatures::Raytracing) == false)
    {
        throw Falcor::RuntimeError("Device does not support raytracing!");
    }

    loadScene(kDefaultScene, getTargetFbo().get());

    JPH::RegisterDefaultAllocator();
    JPH::Factory::sInstance = new JPH::Factory();
    JPH::RegisterTypes();
    mTempAllocator = new JPH::TempAllocatorImpl(10 * 1024 * 1024);
    mJobSystem = new JPH::JobSystemThreadPool(2048, 8, std::thread::hardware_concurrency() - 1);
    mBroadPhaseLayerInterface = new JPHUtil::BPLayerInterfaceImpl();
    mObjectVsBroadPhaseLayerFilter = new JPHUtil::ObjectVsBroadPhaseLayerFilterImpl();
    mObjectVsObjectLayerFilter = new JPHUtil::ObjectLayerPairFilterImpl();

    mPhysics = new JPH::PhysicsSystem();
    mPhysics->Init(65536, 0, 1024, 1024, *mBroadPhaseLayerInterface, *mObjectVsBroadPhaseLayerFilter, *mObjectVsObjectLayerFilter);

    TerrainHandler terrainHandler(*mPhysics);
    mVehicle = new VehicleHandler(*mPhysics);

    mFirstFrame = true;
    mPreviousTime = std::chrono::steady_clock::now();
}

void Renderer::onResize(uint32_t width, uint32_t height)
{
    float h = (float)height;
    float w = (float)width;

    if (mpCamera)
    {
        mpCamera->setFocalLength(18);
        float aspectRatio = (w / h);
        mpCamera->setAspectRatio(aspectRatio);
    }

    mpRtOut = Falcor::Texture::create2D(
        getDevice(), width, height, Falcor::ResourceFormat::RGBA16Float, 1, 1, nullptr,
        Falcor::Resource::BindFlags::UnorderedAccess | Falcor::Resource::BindFlags::ShaderResource
    );
}

void Renderer::onFrameRender(Falcor::RenderContext* pRenderContext, const Falcor::ref<Falcor::Fbo>& pTargetFbo)
{
    pRenderContext->clearFbo(pTargetFbo.get(), kClearColor, 1.0f, 0, Falcor::FboAttachmentType::All);

    if (mpScene)
    {
        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<float> deltaTime;
        if (mFirstFrame)
        {
            deltaTime = std::chrono::duration_cast<std::chrono::duration<float>>(currentTime - currentTime);
            mFirstFrame = false;
        }
        else 
            deltaTime = std::chrono::duration_cast<std::chrono::duration<float>>(currentTime - mPreviousTime);
		mPreviousTime = currentTime;
        
        mPhysics->Update(deltaTime.count(), 1, mTempAllocator, mJobSystem);
        mVehicle->Update(*mPhysics, mKeyHandler.Fwd, mKeyHandler.Bwd, mKeyHandler.Lft, mKeyHandler.Rht);
        
        JPH::RVec3 wheelFwdLeftPos = mVehicle->GetWheel(VehicleHandler::EWheel::FrontLeft)->GetBody2()->GetPosition();
        JPH::Quat wheelFwdLeftRot = mVehicle->GetWheel(VehicleHandler::EWheel::FrontLeft)->GetBody2()->GetRotation();
        JPH::RVec3 wheelFwdRightPos = mVehicle->GetWheel(VehicleHandler::EWheel::FrontRight)->GetBody2()->GetPosition();
        JPH::Quat wheelFwdRightRot = mVehicle->GetWheel(VehicleHandler::EWheel::FrontRight)->GetBody2()->GetRotation();
        JPH::RVec3 wheelRearLeftPos = mVehicle->GetWheel(VehicleHandler::EWheel::RearLeft)->GetBody2()->GetPosition();
        JPH::Quat wheelRearLeftRot = mVehicle->GetWheel(VehicleHandler::EWheel::RearLeft)->GetBody2()->GetRotation();
        JPH::RVec3 wheelRearRightPos = mVehicle->GetWheel(VehicleHandler::EWheel::RearRight)->GetBody2()->GetPosition();
        JPH::Quat wheelRearRightRot = mVehicle->GetWheel(VehicleHandler::EWheel::RearRight)->GetBody2()->GetRotation();
        JPH::RVec3 chassisPos = mVehicle->GetChassis()->GetPosition();
        JPH::Quat chassisRot = mVehicle->GetChassis()->GetRotation();
        
		mVehicle->UpdateVehicle(wheelFwdLeftPos, wheelFwdLeftRot, wheelFwdRightPos, wheelFwdRightRot, wheelRearLeftPos, wheelRearLeftRot, wheelRearRightPos, wheelRearRightRot, chassisPos, chassisRot);

        UpdateMesh(1, *mVehicle->GetPartTransform(VehicleHandler::EWheel::FrontLeft));
		UpdateMesh(2, *mVehicle->GetPartTransform(VehicleHandler::EWheel::FrontRight));
		UpdateMesh(3, *mVehicle->GetPartTransform(VehicleHandler::EWheel::RearLeft));
		UpdateMesh(4, *mVehicle->GetPartTransform(VehicleHandler::EWheel::RearRight));
		UpdateMesh(5, *mVehicle->GetPartTransform(VehicleHandler::EWheel::Num));

        mpCamera->setPosition(mVehicle->cameraAnchor);
        mpCamera->setTarget(mVehicle->cameraTargetAnchor);
        
        Falcor::Scene::UpdateFlags updates = mpScene->update(pRenderContext, getGlobalClock().getTime());

        if (mRayTrace)
            renderRT(pRenderContext, pTargetFbo);
        else
            renderRaster(pRenderContext, pTargetFbo);
    }

    getTextRenderer().render(pRenderContext, getFrameRate().getMsg(), pTargetFbo, { 20, 20 });
}

bool Renderer::onKeyEvent(const Falcor::KeyboardEvent& keyEvent)
{
    if (keyEvent.key == Falcor::Input::Key::Space && keyEvent.type == Falcor::KeyboardEvent::Type::KeyPressed)
    {
        mRayTrace = !mRayTrace;
        return true;
    }

    if (mpScene)
    {
        mKeyHandler.HandleEvent(keyEvent);
        return true;
    }

    return false;
}

void Renderer::loadScene(const std::filesystem::path& path, const Falcor::Fbo* pTargetFbo)
{    
    mpScene = Falcor::Scene::create(getDevice(), path);
    mpCamera = mpScene->getCamera();

    mpScene->setIsAnimated(true);
	mpScene->toggleAnimations(true);
    mpScene->setIsLooped(false);

    // Update the controllers
    float radius = mpScene->getSceneBounds().radius();
    mpScene->setCameraSpeed(radius * 0.25f);
    float nearZ = std::max(0.1f, radius / 750.0f);
    float farZ = radius * 10;
    mpCamera->setDepthRange(nearZ, farZ);
    mpCamera->setAspectRatio((float)pTargetFbo->getWidth() / (float)pTargetFbo->getHeight());

    // Get shader modules and type conformances for types used by the scene.
    // These need to be set on the program in order to use Falcor's material system.
    auto shaderModules = mpScene->getShaderModules();
    auto typeConformances = mpScene->getTypeConformances();

    // Get scene defines. These need to be set on any program using the scene.
    auto defines = mpScene->getSceneDefines();

    // Create raster pass.
    // This utility wraps the creation of the program and vars, and sets the necessary scene defines.
    Falcor::Program::Desc rasterProgDesc;
    rasterProgDesc.addShaderModules(shaderModules);
    rasterProgDesc.addShaderLibrary("../custom-shaders/PerlinRover.3d.slang").vsEntry("vsMain").psEntry("psMain");
    rasterProgDesc.addTypeConformances(typeConformances);

    mpRasterPass = Falcor::RasterPass::create(getDevice(), rasterProgDesc, defines);

    // We'll now create a raytracing program. To do that we need to setup two things:
    // - A program description (RtProgram::Desc). This holds all shader entry points, compiler flags, macro defintions,
    // etc.
    // - A binding table (RtBindingTable). This maps shaders to geometries in the scene, and sets the ray generation and
    // miss shaders.
    //
    // After setting up these, we can create the RtProgram and associated RtProgramVars that holds the variable/resource
    // bindings. The RtProgram can be reused for different scenes, but RtProgramVars needs to binding table which is
    // Scene-specific and needs to be re-created when switching scene. In this example, we re-create both the program
    // and vars when a scene is loaded.

    Falcor::RtProgram::Desc rtProgDesc;
    rtProgDesc.addShaderModules(shaderModules);
    rtProgDesc.addShaderLibrary("../custom-shaders/PerlinRover.rt.slang");
    rtProgDesc.addTypeConformances(typeConformances);
    rtProgDesc.setMaxTraceRecursionDepth(3); // 1 for calling TraceRay from RayGen, 1 for calling it from the
    // primary-ray ClosestHit shader for reflections, 1 for reflection ray
    // tracing a shadow ray
    rtProgDesc.setMaxPayloadSize(24);        // The largest ray payload struct (PrimaryRayData) is 24 bytes. The payload size
    // should be set as small as possible for maximum performance.

    Falcor::ref<Falcor::RtBindingTable> sbt = Falcor::RtBindingTable::create(2, 2, mpScene->getGeometryCount());
    sbt->setRayGen(rtProgDesc.addRayGen("rayGen"));
    sbt->setMiss(0, rtProgDesc.addMiss("primaryMiss"));
    sbt->setMiss(1, rtProgDesc.addMiss("shadowMiss"));
    auto primary = rtProgDesc.addHitGroup("primaryClosestHit", "primaryAnyHit");
    auto shadow = rtProgDesc.addHitGroup("", "shadowAnyHit");
    sbt->setHitGroup(0, mpScene->getGeometryIDs(Falcor::Scene::GeometryType::TriangleMesh), primary);
    sbt->setHitGroup(1, mpScene->getGeometryIDs(Falcor::Scene::GeometryType::TriangleMesh), shadow);

    mpRaytraceProgram = Falcor::RtProgram::create(getDevice(), rtProgDesc, defines);
    mpRtVars = Falcor::RtProgramVars::create(getDevice(), mpRaytraceProgram, sbt);
}

void Renderer::setPerFrameVars(const Falcor::Fbo* pTargetFbo)
{
    auto var = mpRtVars->getRootVar();
    var["PerFrameCB"]["invView"] = inverse(mpCamera->getViewMatrix());
    var["PerFrameCB"]["viewportDims"] = Falcor::float2(pTargetFbo->getWidth(), pTargetFbo->getHeight());
    float fovY = Falcor::focalLengthToFovY(mpCamera->getFocalLength(), Falcor::Camera::kDefaultFrameHeight);
    var["PerFrameCB"]["tanHalfFovY"] = std::tan(fovY * 0.5f);
    var["PerFrameCB"]["sampleIndex"] = mSampleIndex++;
    var["PerFrameCB"]["useDOF"] = mUseDOF;
    var["gOutput"] = mpRtOut;
}

void Renderer::renderRaster(Falcor::RenderContext* pRenderContext, const Falcor::ref<Falcor::Fbo>& pTargetFbo)
{
    FALCOR_ASSERT(mpScene);
    FALCOR_PROFILE(pRenderContext, "renderRaster");

    mpRasterPass->getState()->setFbo(pTargetFbo);
    mpScene->rasterize(pRenderContext, mpRasterPass->getState().get(), mpRasterPass->getVars().get());
}

void Renderer::renderRT(Falcor::RenderContext* pRenderContext, const Falcor::ref<Falcor::Fbo>& pTargetFbo)
{
    FALCOR_ASSERT(mpScene);
    FALCOR_PROFILE(pRenderContext, "renderRT");

    setPerFrameVars(pTargetFbo.get());

    pRenderContext->clearUAV(mpRtOut->getUAV().get(), kClearColor);
    mpScene->raytrace(pRenderContext, mpRaytraceProgram.get(), mpRtVars, Falcor::uint3(pTargetFbo->getWidth(), pTargetFbo->getHeight(), 1));
    pRenderContext->blit(mpRtOut->getSRV(), pTargetFbo->getRenderTargetView(0));
}

void Renderer::UpdateMesh(int nodeID, Falcor::Transform transform)
{
	mpScene->updateNodeTransform(nodeID, transform.getMatrix());
}